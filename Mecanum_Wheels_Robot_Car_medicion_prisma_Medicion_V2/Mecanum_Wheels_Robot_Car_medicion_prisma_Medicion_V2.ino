#include <SoftwareSerial.h>

SoftwareSerial mySerial(A0, A1);  // RX, TX
String BT_value;
String BT_value_temp;
volatile int Front_Distance;
volatile boolean Flag = true;

const int Trig = A3;
const int Echo = A2;
const int PWM2A = 11;      // M1 motor
const int PWM2B = 3;       // M2 motor
const int PWM0A = 6;       // M3 motor
const int PWM0B = 5;       // M4 motor
const int DIR_CLK = 4;
const int DIR_EN = 7;
const int DATA = 8;
const int DIR_LATCH = 12;

// Valores de dirección originales del kit
const int Move_Forward = 216;
const int Move_Backward = 39;
const int Left_Move = 139;
const int Right_Move = 116;
const int Right_Rotate = 106;
const int Left_Rotate = 149;
const int Stop = 0;
const int Upper_Left_Move = 136;
const int Upper_Right_Move = 80;
const int Lower_Left_Move = 3;
const int Lower_Right_Move = 36;
const int Drift_Left = 10;
const int Drift_Right = 20;

// Velocidad por defecto para comandos manuales
int Speed1 = 255;
int Speed2 = 255;
int Speed3 = 255;
int Speed4 = 255;

void Motor(int Dir, int Speed1, int Speed2, int Speed3, int Speed4) {
  analogWrite(PWM2A, Speed1);
  analogWrite(PWM2B, Speed2);
  analogWrite(PWM0A, Speed3);
  analogWrite(PWM0B, Speed4);

  digitalWrite(DIR_LATCH, LOW);
  shiftOut(DATA, DIR_CLK, MSBFIRST, Dir);
  digitalWrite(DIR_LATCH, HIGH);
}

float checkdistance() {
  digitalWrite(Trig, LOW);
  delayMicroseconds(2);
  digitalWrite(Trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(Trig, LOW);

  unsigned long duration = pulseIn(Echo, HIGH, 30000);
  if (duration == 0) {
    return 0;
  }

  return duration / 58.00;
}

// El sensor apunta al lado derecho; el carrito recorre la cara hacia delante.
const int SCAN_PWM = 190;
const float TARGET_MIN_CM = 5.0;
const float TARGET_MAX_CM = 60.0;
const float FORWARD_SPEED_CM_S = 33.0;  // Calibrado inicialmente con una cara de 24.6 cm
const float EDGE_CORRECTION_CM = 2.0;   // Ajustar según el cono acústico y montaje
const unsigned long MAX_SCAN_TIME_MS = 12000;
const byte EDGE_CONFIRMATIONS = 3;

bool targetDetected(float d) {
  return (d >= TARGET_MIN_CM && d <= TARGET_MAX_CM);
}

bool waitForDetectedTarget() {
  unsigned long start = millis();
  while (millis() - start < 5000) {
    if (targetDetected(checkdistance())) {
      Motor(Stop, 0, 0, 0, 0);
      return true;
    }
    delay(30);
  }
  Motor(Stop, 0, 0, 0, 0);
  return false;
}

void Medir_Ancho_Prisma() {
  Serial.println("\n=== MEDICION DEL PRISMA V2 ===");
  mySerial.println("Iniciando medicion...");

  Motor(Stop, 0, 0, 0, 0);
  delay(300);

  if (!waitForDetectedTarget()) {
    Serial.println("ERROR: no se detecto la caja.");
    mySerial.println("Err: Caja no detectada");
    return;
  }

  // 1. Retroceder hasta salir del rango de la cara y localizar su borde inicial.
  Serial.println("Buscando borde inicial en reversa...");
  mySerial.println("Buscando borde inicial...");

  unsigned long startSearch = millis();
  byte lostCount = 0;
  bool initialEdgeFound = false;
  Motor(Move_Backward, SCAN_PWM, SCAN_PWM, SCAN_PWM, SCAN_PWM);

  while (millis() - startSearch < MAX_SCAN_TIME_MS) {
    if (targetDetected(checkdistance())) {
      lostCount = 0;
    } else {
      lostCount++;
      if (lostCount >= EDGE_CONFIRMATIONS) {
        initialEdgeFound = true;
        break;
      }
    }
    delay(30);
  }

  Motor(Stop, 0, 0, 0, 0);
  delay(400);

  if (!initialEdgeFound) {
    mySerial.println("Err: Sin borde inicial");
    return;
  }

  // 2. Avanzar hasta volver a detectar la cara, dejando el sensor dentro del borde.
  unsigned long startReturn = millis();
  byte detectedCount = 0;
  bool targetReacquired = false;
  Motor(Move_Forward, SCAN_PWM, SCAN_PWM, SCAN_PWM, SCAN_PWM);

  while (millis() - startReturn < MAX_SCAN_TIME_MS) {
    if (targetDetected(checkdistance())) {
      detectedCount++;
      if (detectedCount >= EDGE_CONFIRMATIONS) {
        targetReacquired = true;
        break;
      }
    } else {
      detectedCount = 0;
    }
    delay(30);
  }

  Motor(Stop, 0, 0, 0, 0);
  delay(400);

  if (!targetReacquired) {
    mySerial.println("Err: No se recupero la caja");
    return;
  }

  // 3. Recorrer la cara hacia delante hasta detectar su borde final.
  Serial.println("Midiendo cara hacia delante...");
  mySerial.println("Midiendo...");

  unsigned long measureStart = millis();
  unsigned long lastSeenTime = measureStart;
  lostCount = 0;
  bool finalEdgeFound = false;

  Motor(Move_Forward, SCAN_PWM, SCAN_PWM, SCAN_PWM, SCAN_PWM);

  while (millis() - measureStart < MAX_SCAN_TIME_MS) {
    if (targetDetected(checkdistance())) {
      lastSeenTime = millis();
      lostCount = 0;
    } else {
      lostCount++;
      if (lostCount >= EDGE_CONFIRMATIONS) {
        finalEdgeFound = true;
        break;
      }
    }
    delay(30);
  }

  Motor(Stop, 0, 0, 0, 0);

  if (!finalEdgeFound) {
    mySerial.println("Err: Sin borde final");
    return;
  }

  unsigned long elapsed = lastSeenTime - measureStart;
  float widthCm = (elapsed / 1000.0) * FORWARD_SPEED_CM_S - EDGE_CORRECTION_CM;
  if (widthCm < 0) {
    widthCm = 0;
  }

  Serial.print("Tiempo detectando la cara: ");
  Serial.print(elapsed);
  Serial.println(" ms");
  Serial.print("Ancho estimado: ");
  Serial.print(widthCm, 1);
  Serial.println(" cm");

  mySerial.print("Ancho: ");
  mySerial.print(widthCm, 1);
  mySerial.println(" cm");
}

void Ultrasonic_Avoidance() {
  int Front_Distance = checkdistance();
  if (0 < Front_Distance && Front_Distance <= 45) {
    if (Front_Distance <= 20) {
      Flag = !Flag;
      Motor(Stop, 0, 0, 0, 0);
      delay(250);
      Motor(Move_Backward, Speed1, Speed2, Speed3, Speed4);
      delay(200);
      Motor(Stop, 0, 0, 0, 0);
      delay(250);
      if (Flag) {
        Motor(Left_Rotate, Speed1, Speed2, Speed3, Speed4);
      } else {
        Motor(Right_Rotate, Speed1, Speed2, Speed3, Speed4);
      }
      delay(100);
      Motor(Stop, 0, 0, 0, 0);
      delay(250);
    } else {
      Motor(Stop, 0, 0, 0, 0);
      delay(250);
      if (Flag) {
        Motor(Left_Rotate, Speed1, Speed2, Speed3, Speed4);
      } else {
        Motor(Right_Rotate, Speed1, Speed2, Speed3, Speed4);
      }
      delay(100);
      Motor(Stop, 0, 0, 0, 0);
      delay(250);
    }
  } else {
    Motor(Move_Forward, 100, 100, 100, 100);
  }
}

void setup() {
  BT_value = "";
  BT_value_temp = "";
  Front_Distance = 0;

  mySerial.begin(9600);
  Serial.begin(9600);

  pinMode(DIR_CLK, OUTPUT);
  pinMode(DATA, OUTPUT);
  pinMode(DIR_EN, OUTPUT);
  pinMode(DIR_LATCH, OUTPUT);
  pinMode(PWM0B, OUTPUT);
  pinMode(PWM0A, OUTPUT);
  pinMode(PWM2A, OUTPUT);
  pinMode(PWM2B, OUTPUT);
  pinMode(Trig, OUTPUT);
  pinMode(Echo, INPUT);
}

void loop() {
  while (true) {
    while (mySerial.available() > 0) {
      BT_value_temp = BT_value_temp + ((char)(mySerial.read()));
      delay(2);
      if (!mySerial.available() > 0) {
        BT_value = BT_value_temp;
        BT_value_temp = "";
      }
    }
    if (0 < String(BT_value).length()) {
      Serial.println(BT_value);
      if (4 >= String(BT_value).length()) {
        if ('%' == String(BT_value).charAt(0) &&
            '#' == String(BT_value).charAt((String(BT_value).length() - 1))) {
          switch (String(BT_value).charAt(1)) {
            case 'A':
              Motor(Move_Forward, Speed1, Speed2, Speed3, Speed4);
              delay(200);
              BT_value = "";
              break;
            case 'B':
              Motor(Move_Backward, Speed1, Speed2, Speed3, Speed4);
              delay(200);
              BT_value = "";
              break;
            case 'C':
              Motor(Left_Move, Speed1, Speed2, Speed3, Speed4);
              delay(200);
              BT_value = "";
              break;
            case 'D':
              Motor(Right_Move, Speed1, Speed2, Speed3, Speed4);
              delay(200);
              BT_value = "";
              break;
            case 'E':
              Motor(Left_Rotate, Speed1, Speed2, Speed3, Speed4);
              delay(100);
              BT_value = "";
              break;
            case 'F':
              Motor(Right_Rotate, Speed1, Speed2, Speed3, Speed4);
              delay(100);
              BT_value = "";
              break;
            case 'G':
              Motor(Upper_Left_Move, Speed1, Speed2, Speed3, Speed4);
              delay(300);
              BT_value = "";
              break;
            case 'H':
              Motor(Upper_Right_Move, Speed1, Speed2, Speed3, Speed4);
              delay(300);
              BT_value = "";
              break;
            case 'I':
              Motor(Lower_Left_Move, Speed1, Speed2, Speed3, Speed4);
              delay(300);
              BT_value = "";
              break;
            case 'J':
              Motor(Lower_Right_Move, Speed1, Speed2, Speed3, Speed4);
              delay(300);
              BT_value = "";
              break;
            case 'K':
              Motor(Drift_Left, Speed1, Speed2, Speed3, Speed4);
              delay(300);
              BT_value = "";
              break;
            case 'L':
              Motor(Drift_Right, Speed1, Speed2, Speed3, Speed4);
              delay(300);
              BT_value = "";
              break;
            case 'T':
              Ultrasonic_Avoidance();
              break;
            case 'W':
              BT_value = "";
              Medir_Ancho_Prisma();
              break;
            case 'S':
              BT_value = "";
              Motor(Stop, 0, 0, 0, 0);
              break;
          }
        }
      } else {
        BT_value = "";
        Motor(Stop, 0, 0, 0, 0);
      }
    } else {
      Motor(Stop, 0, 0, 0, 0);
    }
  }
}
