#include <QTRSensors.h>
#define enB 5
#define enA 6
#define in1 10
#define in2 9
#define in3 8
#define in4 7

QTRSensors qtr;


float Kp = 1.88; //set up the constants value
float Ki = 0.21;
float Kd = 0.39;
int Pr;
int In;
int De;
int lastEr = 0;

//Increasing the maxspeed can damage the motors - at a value of 255 the 6V motors will receive 7,4 V 
const uint8_t aSpeedM = 55;
const uint8_t bSpeedM = 55;
const uint8_t aSpeedB = 90;
const uint8_t bSpeedB = 90;


const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
boolean mode = LOW; // Low for moter, High for sensor
int sensorFlag = LOW; // Low for sensor calibration , High for Not calibration
const byte interruptPin = 2;
long unsigned int lastPress;
int deboundTime = 40;
boolean ledstate = LOW;
unsigned int ledtime = 0;


void SensorCal() {
  //configure the sensors
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]) {A0, A1, A2, A3, A4, A5, A6, A7}, SensorCount);
                    qtr.setEmitterPin(3);

                    delay(500);
                    pinMode(LED_BUILTIN, OUTPUT);
                    digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode

                    //analogRead() takes about 0.1 ms on an AVR.
                    //0.1 ms per sensor * 4 samples per sensor read (default) * 6 sensors
                    //* 10 reads per calibrate() call =~24 ms per calibrate() call
                    //call alibrate() 400 times to make calibration take about 10 seconds

  for (uint16_t i = 0; i < 400; i++) {
  qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration

  //print the calibration minimumvalues measure when emitters were on
  Serial.begin(9600);
  for (uint8_t i = 0; i < SensorCount; i++) {

  Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibratino maximum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++) {
  Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }

  Serial.println();
  Serial.println();
  delay(1000);
}


void SensorData() {
  // read calibrated sensor valus and obtain a measure of the line position
  // from 0 to 5000 (for a white line, use readLineWhite() instead)

  uint16_t position = qtr.readLineBlack(sensorValues);

  //print the sensor values as numbers from 0 to 1000, where 0 means maximum
  // reflectance and 1000 means minimum reflectance, followed by the line
  //position
  for (uint8_t i = 0; i < SensorCount; i++) {
    Serial.print(sensorValues[i]);
    Serial.print('\t');

  }
  Serial.println(position);
  delay(250);
  if (ledtime > 6) {
    ledstate = !ledstate;
    ledtime = 0;
  }
  digitalWrite(LED_BUILTIN, ledstate);
    ledtime++;

  }


  void forward() {
  analogWrite(enA, 255);
  analogWrite(enB, 255);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);

}

void back() {
  analogWrite(enA, 255);
  analogWrite(enB, 255);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);

}

void right() {
  analogWrite(enA, 255);
  analogWrite(enB, 255);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);

}

void left() {
  analogWrite(enA, 255);
  analogWrite(enB, 255);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);

}

void off() {
  analogWrite(enA, 255);
  analogWrite(enB, 255);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);

}

void motorTest() {
  forward();
  Serial.println("F");
  delay(1000);
  back();
  Serial.println("B");
  delay(1000);
  right();
  Serial.println("R");
  delay(1000);
  left();
  Serial.println("L");
  delay(1000);
  off();
  Serial.println("stop");
  delay(1000);

}

void setup() {
  //put your setup code here, to run once:
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  Serial.begin(9600);
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), changeMode, FALLING);

}

void loop() {

  if (mode == LOW) {
    if (Serial.available() > 0) {
      int inbyte = Serial.read();

      switch (inbyte) {
        case '0':
          Serial.println("L");
          left();
          delay(1000);
          break;
        case '1':
          Serial.println("R");
          right();
          delay(1000);
          break;
        case '2':
          Serial.println("U"); 
          forward();
          delay(1000);
          break;
        case '3':
          Serial.println("D");
          off();
          delay(1000);
          break;

        default:
          break;
      }


    }

  }

  else if (mode == HIGH) {

    if (sensorFlag == LOW) {
      Serial.println("IR Test");
      SensorCal();
      sensorFlag = HIGH;
      Mazecontrol();
    }
    else {
      SensorData();
    }

  }

}


void changeMode() {
  if ((millis() - lastPress) > deboundTime) {
    lastPress = millis();
    if (digitalRead(interruptPin) == 0) {
      mode = !mode;
    }
    else if (digitalRead(interruptPin) == 1) {
      mode = mode;

    }
  }
}


void Mazecontrol() {
  uint16_t position = qtr.readLineBlack(sensorValues);
  int error = 3500 - position;

  Pr = error;
  In = In + error;
  De = error - lastEr;
  lastEr = error;
  
  int Mspeed = Pr*Kp + In*Ki + De*Kd;
  
  int Mspeeda = aSpeedB + Mspeed;
  int Mspeedb = bSpeedB - Mspeed;
  
  if (Mspeeda > aSpeedM) {
    Mspeeda = aSpeedM;
  }
  if (Mspeedb > bSpeedM) {
    Mspeedb = bSpeedM;
  }
  if (Mspeeda < 0) {
    Mspeeda = 0;
  }
  if (Mspeedb < 0) {
    Mspeedb = 0;
  } 
  // Serial.print(motorspeed);Serial.print(" ");Serial.print(motorspeeda);Serial.print(" ");Serial.println(motorspeedb);
    off();
  
}
