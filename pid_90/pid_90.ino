// ==== Auto Seguidor de Linea con PID ====

// ==== Pines motor izquierdo (Motor 1) ====
const int in1 = 13;
const int in2 = 12;
const int ena = 11; // PWM - velocidad motor izquierdo

// ==== Pines motor derecho (Motor 2) ====
const int in3 = 8;
const int in4 = 3;
const int enb = 5; // PWM - velocidad motor derecho

// ==== Pines sensores IR ====
const int leftSensor = 2;
const int centerSensor = 4;
const int rightSensor = 7;

// ==== Parametros PID ====
const float kp = 17.0; // Proporcional
const float ki = 0.1;  // Integral - para corregir errores acumulados
const float kd = 12.0;  // Derivativo - para suavizar la respuesta

// ==== Variables de Control PID ====
float integral = 0.0;
int previousError = 0;

// Velocidad base para ambos motores/ruedas
const int baseSpeed = 135;

// Variable para guardar la ultima lectura valida de sensores (no 000)
int lastValidSensors = 0b111;

void setup() {
  // Configurar sensores como entrada
  pinMode(leftSensor, INPUT);
  pinMode(centerSensor, INPUT);
  pinMode(rightSensor, INPUT);
  
  // Configurar motores como salida
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(ena, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(enb, OUTPUT);

  // Inicializar direccion de los motores
  initializeMotorsDirection();
}

void loop() {
  // ==== Lectura de sensores ====
  int sensorsData = readSensors();

  // ==== Calculo de error de posicion ====
  int currentError = calculateError(sensorsData);

  // ==== Control PID ====
  float correction = PID(currentError);
  
  // ==== Calculo de velocidades y movimiento ====
  int leftSpeed, rightSpeed;
  calculateSpeeds(correction, leftSpeed, rightSpeed);
  drive(leftSpeed, rightSpeed);
}

// Inicializar direccion de motores hacia adelante
void initializeMotorsDirection() {
  // Motor izquierdo (Motor 1)
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  
  // Motor derecho (Motor 2)
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

int readSensors() {
  // Leer sensores IR digitales
  // Linea Negra devuelve 1, Superficie Blanca devuelve 0
  int Ls = digitalRead(leftSensor);
  int Cs = digitalRead(centerSensor);
  int Rs = digitalRead(rightSensor);

  // Codificar lectura de sensores en formato binario: Ls Cs Rs (0bXYZ)
  int sensorsData = (Ls << 2) | (Cs << 1) | Rs;

  // Guardar lectura como la ultima valida para reaccionar en caso de perder la linea
  if (sensorsData != 0b000) {
    lastValidSensors = sensorsData;
  }

  return sensorsData;
}

int calculateError(int sensorsData) {
  // Calcular error basado en la posicion de la linea
  // Error positivo: linea a la izquierda, entonces girar a la izquierda
  // Error negativo: linea a la derecha, entonces girar a la derecha
  // Error = 0: linea centrada, entonces seguir recto
  switch (sensorsData) {
    case 0b100: return  4; // linea a la izquierda
    case 0b110: return  2; // linea levemente a la izquierda
    case 0b010: return  0; // centrado
    case 0b011: return -2; // linea levemente a la derecha
    case 0b001: return -4; // linea a la derecha
    case 0b000:
      // Si perdio la linea, decidir giro segun ultima lectura valida
      if (lastValidSensors == 0b100 || lastValidSensors == 0b110) { // Ultima linea a la izquierda
        return  12;
      } else if (lastValidSensors == 0b001 || lastValidSensors == 0b011) { // Ultima linea a la derecha
        return -12;
      } else { // En otro caso, mantener el error anterior
        return previousError;
      }
    default: return 0;
  }
}

// ==== Algoritmo PID ====
// Dado que el loop() se ejecuta a una frecuencia constante, no se involucra al tiempo
// explicitamente en los terminos integral y derivativo, evitando asi tambien que el
// componente derivativo se dispare por variaciones muy pequeñas en el tiempo
float PID(int currentError) {
  // Sumar el error actual a la acumulacion total (componente integral)
  // Ayuda a corregir errores sistematicos o sesgos en el sistema
  integral += currentError;
  
  // Prevenir el "integral wind-up" limitando el valor acumulado
  integral = constrain(integral, -100, 100);
  
  // Calcular el cambio en el error desde la ultima iteracion (componente derivativo)
  // Suaviza la respuesta ante cambios bruscos en el error
  float derivative = currentError - previousError;
  
  // Calcular la correccion total del PID
  float correction = kp * currentError + ki * integral + kd * derivative;
  
  // Guardar el error actual para la proxima iteracion
  previousError = currentError;

  return correction;
}

// El simbolo & indica que las variables se pasan por referencia
void calculateSpeeds(float correction, int &leftSpeed, int &rightSpeed) {
  // Calcular velocidades de motores
  leftSpeed = (int) round(baseSpeed - correction);
  rightSpeed = (int) round(baseSpeed + correction);
  
  // Aplicar limites de velocidad en el rango valido PWM
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);
}

// Indicar velocidades de motores calculadas
void drive(int leftSpeed, int rightSpeed) {
  // Motor izquierdo (Motor 1)
  analogWrite(ena, leftSpeed);
  
  // Motor derecho (Motor 2)
  analogWrite(enb, rightSpeed);
}