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

// Valores de direccion originales del kit
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

const int SCAN_PWM = 190;
const int ROTATION_PWM = 130;
const unsigned long TURN_90_MS = 1400;
const unsigned long CLEAR_CORNER_MS = 250;
const float TARGET_MIN_CM = 5.0;
const float TARGET_MAX_CM = 60.0;
const float FORWARD_SPEED_CM_S = 33.0;  // Calibrado inicialmente con una cara de 24.6 cm
const float EDGE_CORRECTION_CM = 2.0;
const unsigned long MAX_SCAN_TIME_MS = 12000;
const unsigned long MIN_FACE_SPAN_MS = 200;
const byte EDGE_CONFIRMATIONS = 3;

bool targetDetected(float d) {
  return (d >= TARGET_MIN_CM && d <= TARGET_MAX_CM);
}

bool scanFace(float &widthCm) {
  Motor(Stop, 0, 0, 0, 0);
  delay(300);

  // Avanzar, confirmar el primer borde y continuar hasta confirmar el segundo.
  unsigned long scanStart = millis();
  unsigned long measureStart = 0;
  unsigned long lastSeenTime = 0;
  byte detectedCount = 0;
  byte lostCount = 0;
  bool measuringFace = false;
  bool finalEdgeFound = false;
  Motor(Move_Forward, SCAN_PWM, SCAN_PWM, SCAN_PWM, SCAN_PWM);

  while (millis() - scanStart < MAX_SCAN_TIME_MS) {
    float distance = checkdistance();
    unsigned long now = millis();

    if (!measuringFace) {
      if (targetDetected(distance)) {
        if (detectedCount == 0) {
          measureStart = now;
        }
        detectedCount++;
        if (detectedCount >= EDGE_CONFIRMATIONS) {
          measuringFace = true;
          lastSeenTime = now;
          lostCount = 0;
        }
      } else {
        detectedCount = 0;
      }
    } else if (targetDetected(distance)) {
      lastSeenTime = now;
      lostCount = 0;
    } else {
      lostCount++;
      if (lostCount >= EDGE_CONFIRMATIONS) {
        if (lastSeenTime - measureStart >= MIN_FACE_SPAN_MS) {
          finalEdgeFound = true;
          break;
        }

        // Ignorar ecos breves de esquina y seguir buscando una cara completa.
        Serial.println("Eco corto; buscando de nuevo el primer borde...");
        measuringFace = false;
        detectedCount = 0;
        lostCount = 0;
      }
    }
    delay(30);
  }

  Motor(Stop, 0, 0, 0, 0);

  if (!finalEdgeFound) {
    mySerial.println("Err: No se detectaron 2 bordes");
    Serial.println("ERROR: no se detectaron dos bordes de la cara.");
    return false;
  }

  unsigned long elapsed = lastSeenTime - measureStart;
  widthCm = (elapsed / 1000.0) * FORWARD_SPEED_CM_S - EDGE_CORRECTION_CM;
  if (widthCm < 0) {
    widthCm = 0;
  }

  return true;
}

void turnRight90() {
  Motor(Move_Forward, SCAN_PWM, SCAN_PWM, SCAN_PWM, SCAN_PWM);
  delay(CLEAR_CORNER_MS);
  Motor(Stop, 0, 0, 0, 0);
  delay(200);

  Motor(Right_Rotate, ROTATION_PWM, ROTATION_PWM, ROTATION_PWM, ROTATION_PWM);
  delay(TURN_90_MS);
  Motor(Stop, 0, 0, 0, 0);
  delay(400);
}

void Medir_4_Caras() {
  float widths[4];

  Serial.println("\n=== MEDICION DE LAS 4 CARAS ===");
  mySerial.println("Iniciando 4 caras...");

  for (byte face = 0; face < 4; face++) {
    if (face > 0) {
      Serial.print("Girando hacia cara ");
      Serial.println(face + 1);
      mySerial.print("Buscando cara ");
      mySerial.println(face + 1);

      turnRight90();
    }

    Serial.print("Midiendo cara ");
    Serial.println(face + 1);
    mySerial.print("Midiendo cara ");
    mySerial.println(face + 1);

    if (!scanFace(widths[face])) {
      Motor(Stop, 0, 0, 0, 0);
      return;
    }

    Serial.print("Cara ");
    Serial.print(face + 1);
    Serial.print(": ");
    Serial.print(widths[face], 1);
    Serial.println(" cm");

    mySerial.print("Cara ");
    mySerial.print(face + 1);
    mySerial.print(": ");
    mySerial.print(widths[face], 1);
    mySerial.println(" cm");
  }

  Motor(Stop, 0, 0, 0, 0);
  Serial.println("Medicion de las 4 caras completada.");
  mySerial.println("4 caras completadas");
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
              Medir_4_Caras();
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
