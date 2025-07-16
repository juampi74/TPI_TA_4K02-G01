// ==== Auto Seguidor de Línea con PID ====

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

// ==== Variables PID ====
float kp = 15.0;  // Proporcional
float ki = 0.3;  // Integral - para corregir errores acumulados
float kd = 5.0;  // Derivativo - para suavizar la respuesta

// Variables de control
float integral = 0.0;
float previousError = 0.0;
unsigned long previousTime = 0;

int baseSpeed = 135;  // Velocidad base como la mitad de la máxima

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
  
  printStartupInfo();
  
  previousTime = millis();
}

void loop() {
  int Ls, Cs, Rs;
  int sensorsData = readSensors(Ls, Cs, Rs);
  
  int currentError = calculateError(sensorsData, previousError);
  
  float correction = PID(currentError);
  
  int leftSpeed, rightSpeed;
  calculateSpeeds(correction, leftSpeed, rightSpeed);
  
  // Ejecutar movimiento
  drive(leftSpeed, rightSpeed);
  
  debugInfo(Ls, Cs, Rs, sensorsData, currentError, correction, leftSpeed, rightSpeed);
}

void printStartupInfo() {
  // Inicializar comunicación serial
  Serial.begin(9600);
  Serial.println("======================================================================");
  Serial.println("=================== Auto Seguidor de Linea Iniciado ==================");
  Serial.println("======================================================================");
  Serial.print("            Variables PID: kp = ");
  Serial.print(kp, 2);
  Serial.print(" | ki = ");
  Serial.print(ki, 2);
  Serial.print(" | kd = ");
  Serial.println(kd, 2);
  Serial.println("======================================================================");
  Serial.println("Formato de datos:");
  Serial.println(" - S: sensores (izq, cen, der) - 0 = linea negra, 1=superficie blanca");
  Serial.println(" - E: valor de error");
  Serial.println(" - C: correccion PID aplicada");
  Serial.println(" - VL: velocidad motor izquierdo");
  Serial.println(" - VR: velocidad motor derecho");
  Serial.println(" - [RECTO]/[IZQUIERDA]/[DERECHA]: direccion corregida del auto");
  Serial.println("======================================================================");
}

// El simbolo & indica que las variables se pasan por referencia
int readSensors(int &Ls, int &Cs, int &Rs) {
  // Leer sensores IR digitales
  // Ahora sí 0 = línea negra, 1 = superficie blanca
  Ls = !digitalRead(leftSensor);
  Cs = !digitalRead(centerSensor);
  Rs = !digitalRead(rightSensor);
  
  // Codificar sensores en formato binario: sI sC sD (0bXXX)
  return (Ls << 2) | (Cs << 1) | Rs;
}

int calculateError(int sensorsData, int previousErrorLocal) {
  // Calcular error basado en la posición de la línea
  // Error positivo: linea a la izquierda → girar izquierda
  // Error negativo: linea a la derecha → girar derecha
  // Error = 0: linea centrada → seguir recto
  int errorLocal;
  switch (sensorsData) {
    case 0b011: errorLocal = 4; break; // linea a la izquierda
    case 0b001: errorLocal = 2; break; // linea levemente a la izquierda
    case 0b101: errorLocal = 0;  break; // centrado
    case 0b100: errorLocal = -2;  break; // linea levemente a la derecha
    case 0b110: errorLocal = -4;  break; // linea a la derecha
    case 0b111: errorLocal = previousErrorLocal; break; 
    default: errorLocal = 0; break;
  }
  return errorLocal;
}

float PID(int currentError) {
  // Algoritmo PID
  
  unsigned long currentTime = millis();
  float elapsedTime = (currentTime - previousTime) / 1000.0;
  if (elapsedTime <= 0.0) elapsedTime = 0.001;
  
  // Acumular el error ponderado por el tiempo transcurrido
  // Esto representa el área bajo la curva del error, útil para corregir errores sistemáticos
  integral += currentError * elapsedTime;
  
  // Prevenir el "integral wind-up" limitando el valor acumulado
  integral = constrain(integral, -100, 100);
  
  // Calcular la derivada: que tan rapido cambia el error
  float derivative = (currentError - previousError) / elapsedTime;
  
  // Calcular correccion total del PID
  float correction = kp * currentError + ki * integral + kd * derivative;
  
  // Actualizar estados para la próxima iteración
  previousError = currentError;
  previousTime = currentTime;

  return correction;
}

void calculateSpeeds(float correction, int &leftSpeed, int &rightSpeed) {
  // Calcular velocidades de motores
  leftSpeed = baseSpeed - correction;
  rightSpeed = baseSpeed + correction;
  
  // Aplicar limites y velocidad minima
  leftSpeed = constrain(leftSpeed, 120, 255);
  rightSpeed = constrain(rightSpeed, 120, 255);
}

// Funcion para controlar los motores
void drive(int leftSpeed, int rightSpeed) {
  // Motor izquierdo (Motor 1)
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(ena, leftSpeed);
  
  // Motor derecho (Motor 2)
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enb, rightSpeed);
}

void stopMotors() {
  // Motor izquierdo
  digitalWrite(in1, HIGH);
  digitalWrite(in2, HIGH);
  analogWrite(ena, 0);

  // Motor derecho
  digitalWrite(in3, HIGH);
  digitalWrite(in4, HIGH);
  analogWrite(enb, 0);
}

// Funcion para mostrar información de debug
void debugInfo(int Ls, int Cs, int Rs, int sensorsData, int error, float correction, int leftSpeed, int rightSpeed) {
  Serial.print("S:");
  Serial.print(Ls);
  Serial.print(Cs);
  Serial.print(Rs);
  Serial.print(" (");
  Serial.print(sensorsData, BIN);
  Serial.print(")");
  
  Serial.print(" | E:");
  Serial.print(error);
  
  Serial.print(" | C:");
  Serial.print(correction, 1);
  
  Serial.print(" | VL:");
  Serial.print(leftSpeed);
  
  Serial.print(" | VR:");
  Serial.print(rightSpeed);
  
  // Indicar direccion de movimiento
  if (error < 0) {
    Serial.print(" [IZQUIERDA]");
  } else if (error > 0) {
    Serial.print(" [DERECHA]");
  } else {
    Serial.print(" [RECTO]");
  }
  
  Serial.println();
}