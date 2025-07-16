// ==== Auto Seguidor de Línea con PID + Búsqueda de Línea ====

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

// ==== Variables de control PID ====
float integral = 0.0;
float previousError = 0.0;
unsigned long previousTime = 0;

int baseSpeed = 135;  // Velocidad base como la mitad de la máxima

// ==== Variables de búsqueda de línea ====
int searchDirection = 0;   // 1 = izquierda, -1 = derecha
int searchPhase = 0;       // 0: giro sobre eje, 1: giro con avance
int searchStep = 0;        // 0: giro eje lado opuesto, 1: avance+giro lado opuesto, 2: giro eje otro lado, 3: avance+giro otro lado 
bool searchFailed = false; // Indica si se ha fallado en encontrar la línea y corresponde detener el auto 
int opposite_side = 0;     // Lado opuesto al último error
int same_side = 0;         // Lado igual al último error

unsigned long searchCycleStartTime = 0; // Inicio del ciclo actual de la búsqueda
unsigned long searchTotalStartTime = 0; // Inicio total de la búsqueda

const int SEARCH_TURN_SPEED = 175; // Velocidad de giro
const int SOFT_FORWARD_SPEED = 125; // Velocidad de avance leve en fase 1

const unsigned long SEARCH_CYCLE_TIMEOUT = 2000; // Tiempo entre ciclos de búsqueda
const unsigned long SEARCH_TOTAL_TIMEOUT = 10000; // Detener búsqueda después de cierto tiempo sin encontrar la línea

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
  
  // Inicializar tiempo para PID
  previousTime = millis();
}

void loop() {
  // Verificar si se ha excedido el tiempo de búsqueda de la línea
  if (searchFailed) {
    stopMotors();
    return;
  }

  int Ls, Cs, Rs;
  int sensorsData = readSensors(Ls, Cs, Rs);

  if (sensorsData == 0b000) {
    // Modo búsqueda
    searchLine();
  } else {
    resetSearchState();

    // Modo PID normal
    int currentError = calculateError(sensorsData);
    float correction = PID(currentError);
    int leftSpeed, rightSpeed;
    calculateSpeeds(correction, leftSpeed, rightSpeed);
    drive(leftSpeed, rightSpeed);
  }
}

// El símbolo & indica que las variables se pasan por referencia
int readSensors(int &Ls, int &Cs, int &Rs) {
  // Leer sensores IR digitales (1: línea negra, 0: superficie blanca)
  Ls = digitalRead(leftSensor);
  Cs = digitalRead(centerSensor);
  Rs = digitalRead(rightSensor);

  // Convertir a formato binario: 0bXYZ donde X=izq, Y=centro, Z=derecha
  return (Ls << 2) | (Cs << 1) | Rs;
}

int calculateError(int sensorsData) {
  // Calcular error basado en la posición de la línea
  // Error positivo: linea a la izquierda → girar izquierda
  // Error negativo: linea a la derecha → girar derecha
  // Error = 0: linea centrada → seguir recto
  switch (sensorsData) {
    case 0b100: return  4;  // línea a la izquierda
    case 0b110: return  2;  // línea ligeramente a la izquierda
    case 0b010: return  0;  // línea centrada
    case 0b011: return -2;  // línea ligeramente a la derecha
    case 0b001: return -4;  // línea muy a la derecha
    default:    return  0;
  }
}

float PID(int currentError) {
  unsigned long currentTime = millis();
  float elapsedTime = (currentTime - previousTime) / 1000.0; // Convertir a segundos
  if (elapsedTime <= 0.0) elapsedTime = 0.001; // Evitar división por cero

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
  leftSpeed  = (int) round(baseSpeed - correction);
  rightSpeed = (int) round(baseSpeed + correction);

  // Limitar las velocidades al rango válido
  leftSpeed = constrain(leftSpeed, 120, 255);
  rightSpeed = constrain(rightSpeed, 120, 255);
}

void drive(int leftSpeed, int rightSpeed) {
  digitalWrite(in1, HIGH); digitalWrite(in2, LOW); analogWrite(ena, leftSpeed);  // Motor izquierdo
  digitalWrite(in3, HIGH); digitalWrite(in4, LOW); analogWrite(enb, rightSpeed); // Motor derecho
}

void searchLine() {
  updateSearchState();

  if (searchFailed) return;

  executeSearchMovement();
}

void updateSearchState() {
  unsigned long currentTime = millis();

  // Inicialización de variables al entrar por primera vez
  if (searchTotalStartTime == 0) {
    searchTotalStartTime = currentTime;
    searchCycleStartTime = currentTime;
    searchStep = 0;

    // Calcular lados una sola vez al iniciar la búsqueda
    opposite_side = (previousError > 0) ? -1 : 1; // lado opuesto al último error
    same_side = -opposite_side;

    // Primer paso: giro sobre el eje hacia el lado opuesto al último error
    searchPhase = 0;
    searchDirection = opposite_side;
  }

  // Si se excede el tiempo total de búsqueda → detener
  if (currentTime - searchTotalStartTime > SEARCH_TOTAL_TIMEOUT) {
    stopMotors();
    searchFailed = true;
    return;
  }
  
  // Modificar dirección y fase después de un ciclo de búsqueda
  if (currentTime - searchCycleStartTime > SEARCH_CYCLE_TIMEOUT) {
    searchStep = (searchStep + 1) % 4;
    searchCycleStartTime = currentTime;

    switch (searchStep) {
      case 0: searchPhase = 0; searchDirection = opposite_side; break; // Giro eje lado opuesto
      case 1: searchPhase = 1; searchDirection = opposite_side; break; // Avance+giro lado opuesto
      case 2: searchPhase = 0; searchDirection = same_side; break; // Giro eje para el otro lado
      case 3: searchPhase = 1; searchDirection = same_side; break; // Avance+giro para el otro lado
    }
  }
}

void executeSearchMovement() {
  if (searchPhase == 0) {
    // Fase 0: Giro sobre el eje
    if (searchDirection > 0) { // Girar a la izquierda
      digitalWrite(in1, LOW);  digitalWrite(in2, HIGH); analogWrite(ena, SEARCH_TURN_SPEED);
      digitalWrite(in3, HIGH); digitalWrite(in4, LOW);  analogWrite(enb, SEARCH_TURN_SPEED);
    } else { // Girar a la derecha
      digitalWrite(in1, HIGH); digitalWrite(in2, LOW);  analogWrite(ena, SEARCH_TURN_SPEED);
      digitalWrite(in3, LOW);  digitalWrite(in4, HIGH); analogWrite(enb, SEARCH_TURN_SPEED);
    }
  } else {
    // Fase 1: Giro con avance suave
    if (searchDirection > 0) { // Girar a la izquierda con avance
      digitalWrite(in1, HIGH); digitalWrite(in2, LOW);  analogWrite(ena, SOFT_FORWARD_SPEED);
      digitalWrite(in3, HIGH); digitalWrite(in4, LOW);  analogWrite(enb, SEARCH_TURN_SPEED);
    } else { // Girar a la derecha con avance
      digitalWrite(in1, HIGH); digitalWrite(in2, LOW);  analogWrite(ena, SEARCH_TURN_SPEED);
      digitalWrite(in3, HIGH); digitalWrite(in4, LOW);  analogWrite(enb, SOFT_FORWARD_SPEED);
    }
  }
}

void resetSearchState() {
  // Reiniciar estado de búsqueda si se detecta nuevamente la línea
  searchFailed = false;
  searchTotalStartTime = 0; // Reiniciar tiempo total de búsqueda
  searchCycleStartTime = 0; // Reiniciar tiempo del ciclo actual
  searchStep = 0; // Reiniciar paso de búsqueda
}

void stopMotors() {
  // Detener ambos motores
  digitalWrite(in1, HIGH); digitalWrite(in2, HIGH); analogWrite(ena, 0);
  digitalWrite(in3, HIGH); digitalWrite(in4, HIGH); analogWrite(enb, 0);
}