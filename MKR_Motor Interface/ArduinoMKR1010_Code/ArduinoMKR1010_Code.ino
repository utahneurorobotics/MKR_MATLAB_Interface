/*
   The following code is designed for the ECE 3960 Course

   This code receives UDP Packets and parses them to get
   read and write to the arduino board.

   Connor Olsen 2021
   NeuroRobotics Lab
   University of Utah
*/

#include <WiFiNINA.h>              // Click here to get the library: http://librarymanager/All#WiFiNINA
#include <MKRMotorCarrier.h>       // Click here to get the library: http://librarymanager/All#MKRMotorCarrier
//#include "SparkFun_MMA8452Q.h"     // Click here to get the library: http://librarymanager/All#SparkFun_MMA8452Q
#include "Adafruit_TCS34725.h"     // Click here to get the library: http://librarymanager/ALL#Adafruit_TCS34725
#include <QTRSensors.h>            // Click here to get the library: http://librarymanager/All#QTRSensors 
#include <avdweb_AnalogReadFast.h> // Click here to get the library: http://librarymanager/All#avdweb_AnalogReadFast 
#include <WiFiUdp.h>
#include <SPI.h>
#include <Wire.h>
#include "rgb_led.h"
#include "udp_access_point.h"
#include <Adafruit_MPU6050.h> // Click here to get the library: http://librarymanager/All#Adafruit_MPU6050

#define HERTZ 10000 // PERIOD in MICROSECONDS
#define SERIALDEBUG 1

int getCommand(String input);
double getPin(String input);
int getVal(String input);
void executeCommand(String udpPacket);
void udpSend(char* input);
void resetReadings();

int status = WL_IDLE_STATUS;

char packetBuffer[256];

bool streamAnalogData = false; // flag to stream the analog data continuously
bool streamIMUData = false;    // flag to stream the IMU data continuously
bool streamIRData = false;     // flag to stream the IR data continuously
bool streamULTRAData = false;  // flag to stream the ultrasonic data continuously
int a1;
int a2;
int a5;
int a6;
double t;
double x;
double y;
double z;
float red, green, blue;

//Ultrasonic sensor declarations
int trigPin = 0;
int echoPin = 1;
unsigned long duration;

//Piezo Piano
int tonePeriod;
unsigned long toneDuration;

// create the WiFi-UDP object
udp_access_point * wifi;

// create rgb LED object
rgb_led led;
Adafruit_MPU6050 mpu;                   // create instance of the MPU6050 class


//Create RGB Sensor
Adafruit_TCS34725 rgbSensor = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

// Reflectance sensor Setup variables
QTRSensors qtr;
const uint8_t SensorCount = 4;
uint16_t sensorValues[SensorCount];


void setup() {
  delay(500);

    // set accelerometer range to +-8G
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  // set gyro range to +- 500 deg/s
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  // set filter bandwidth to 21 Hz
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A5, INPUT);
  pinMode(A6, INPUT);

  led.init();
  led.yellow(50);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  Serial.begin(9600);
  //  while (!Serial);

  wifi = new udp_access_point(551, IPAddress(192, 168, 1, 100));
  if (wifi->isReady()) {
    led.blue(50);
    Serial.println("Success!");
    Serial.println("WiFi Initialized");
  }

  // Check for Motor Carrier
  if (controller.begin()) {
    controller.reboot();
    M1.setDuty(0);
    M2.setDuty(0);
    M3.setDuty(0);
    M4.setDuty(0);
    Serial.println("Motor Carrier Initialized");
  }
  else {
    Serial.println("Motor Carrier Not Detected");
    while (1); //hang if not connected
  }

  // Check for Inertial Measurement Unit
  if (mpu.begin() == false) {
    Serial.println("IMU Offline");
  }  else {
    Serial.println("IMU Online");
  }

  // Check for RGB Sensor
  if (rgbSensor.begin() == false) {
    Serial.println("RGB Sensor Offline");
  } else {
    Serial.println("RGB Sensor Online");
  }

  // Turn on Blue LED, indicating Initialization
  // completed and the MKR is ready to receive signals
  led.blue(100);
}

void loop() {
  double start_t = micros();

  // if there's data available, read a packet
  if (wifi->checkForPacket()) {
    if (SERIALDEBUG) {
      Serial.println("Contents: ");
      Serial.println(wifi->getPacket());
    }
    executeCommand(wifi->getPacket());
  }

  // If either data stream flags are enabled, stream appropriate data
  if (streamAnalogData || streamIMUData || streamIRData || streamULTRAData) {
    resetReadings();
    char analogReturn[100];
    if (streamAnalogData) {
      a1 = analogReadFast(A1);
      a2 = analogReadFast(A2);
      a5 = analogReadFast(A5);
      a6 = analogReadFast(A6);
    }
    if (streamIMUData) {
      sensors_event_t accel, gyro, temp;
      mpu.getEvent(&accel, &gyro, &temp);
      x = accel.acceleration.x;
      y = accel.acceleration.y;
      z = accel.acceleration.z; // In m/s^2
    }
    if (streamIRData) {
      qtr.read(sensorValues);
    }
    if (streamULTRAData) {
      digitalWrite(trigPin, LOW);  //LOW or HIGH
      delayMicroseconds(5);
      digitalWrite(trigPin, HIGH); //LOW or HIGH
      delayMicroseconds(10); //How long should the trigger pulse be?
      digitalWrite(trigPin, LOW); //LOW or HIGH
      duration = pulseIn(echoPin, HIGH, 60000);
    }
    sprintf(analogReturn, "STR:%d:%d:%d:%d:%f:%f:%f:%d:%d:%d:%d:%d", a1, a2, a5, a6, x, y, z, sensorValues[0], sensorValues[1], sensorValues[2], sensorValues[3], duration);
    wifi->sendPacket(analogReturn);
    //    Serial.println(analogReturn);
  }
  //  if (micros() - start_t < HERTZ)
  //    delayMicroseconds(HERTZ - (micros() - start_t));
  //  Serial.println(micros() - start_t);
}

void executeCommand(String udpPacket) {
  char returnMsg[50];
  switch (getCommand(udpPacket)) {
    // Future commands to implement:
    // Reset
    case 0: { //---Set the Pin Mode---
        pinMode(getPin(udpPacket), getVal(udpPacket));
        //      return 1;
        break;
      }

    case 1: { //---Perform a Digital Write Function---
        digitalWrite(getPin(udpPacket), getVal(udpPacket));
        //      return 1;
        break;
      }

    case 2: //---Perform a Digital Read Function---
      { sprintf(returnMsg, "DIG:%d:%d", (int)getPin(udpPacket), (int)digitalRead(getPin(udpPacket)));
        if (SERIALDEBUG) {
          Serial.println("This is a Digital Read.");
          Serial.println((int)getPin(udpPacket));
          Serial.println((int)digitalRead(getPin(udpPacket)));
          Serial.println(returnMsg);
        }
        wifi->sendPacket(returnMsg);
        break;
      }

    case 3: //---Perform an Analog Write Function---
      { analogWrite(getPin(udpPacket), getVal(udpPacket));
        break;
      }

    case 4: //---Perform and Analog Read function---
      { sprintf(returnMsg, "ANR:%d:%d", (int)getPin(udpPacket), (int)analogReadFast(getPin(udpPacket)));
        wifi->sendPacket(returnMsg);
        break;
      }

    //---Set the onboard LED---
    case 5: {
        String color = String(getPin(udpPacket));
        Serial.println(getPin(udpPacket));
        Serial.println(color);
        int red = color.substring(1, 4).toInt();
        int green = color.substring(4, 7).toInt();
        int blue = color.substring(7).toInt();

        if (SERIALDEBUG) {
          Serial.println(getPin(udpPacket));
          Serial.println(color);
          Serial.print("red = ");
          Serial.print(red);
          Serial.print(" green = ");
          Serial.print(green);
          Serial.print(" blue = ");
          Serial.println(blue);
        }
        led.setTo(red, green, blue);
        break;
      }

    //---Set Motor Carrier Motors---
    case 6: {
        switch ((int)getPin(udpPacket)) {
          case 1:
            M1.setDuty(getVal(udpPacket));
            break;
          case 2:
            M2.setDuty(getVal(udpPacket));
            break;
          case 3:
            M3.setDuty(getVal(udpPacket));
            break;
          case 4:
            M4.setDuty(getVal(udpPacket));
            break;
        }
        break;
      }

    case 7: { //---Set Motor Carrier Servos---
        switch ((int)getPin(udpPacket)) {
          case 1:
            servo1.setAngle(getVal(udpPacket));
            break;
          case 2:
            servo2.setAngle(getVal(udpPacket));
            break;
          case 3:
            servo3.setAngle(getVal(udpPacket));
            break;
          case 4:
            servo4.setAngle(getVal(udpPacket));
            break;
        }
        //      return 1;
        break;
      }

    case 8: //---Sets the Stream Data Flag---
    { switch ((int)getPin(udpPacket)) {
          case 0:
            streamAnalogData = getVal(udpPacket);
            sprintf(returnMsg, "MSG:Analog Stream Set to %d", getVal(udpPacket));
            break;
          case 1:
            streamIMUData = getVal(udpPacket);
            sprintf(returnMsg, "MSG:IMU Stream Set to %d", getVal(udpPacket));
            break;
          case 2:
            streamIRData = getVal(udpPacket);
            sprintf(returnMsg, "MSG:IR Stream set to %d", getVal(udpPacket));
            break;
          case 3:
            streamULTRAData = getVal(udpPacket);
            sprintf(returnMsg, "MSG:ULTRA Stream set to %d", getVal(udpPacket));
            break;
        }
        wifi->sendPacket(returnMsg);
        break;
      }

    case 9: //---Confirm Connection with Client---
      { wifi->sendPacket("1");
        break;
      }

    case 10: //---Return Battery Voltage---
      { double voltage;
        switch (getVal(udpPacket)) {
          case 0:
            voltage = battery.getFiltered();
            break;
          case 1:
            voltage = battery.getConverted();
            break;
        }
        sprintf(returnMsg, "MSG:Current Battery Voltage = %f", voltage);
        wifi->sendPacket(returnMsg);
        break;
      }

    case 11: //---Ultrasonic ranger---
      { // The sensor is triggered by a ??? pulse of ??? or more microseconds.
        // Give a short ??? pulse beforehand to ensure a clean ??? pulse:
        digitalWrite(trigPin, LOW);  //LOW or HIGH
        delayMicroseconds(5);
        digitalWrite(trigPin, HIGH); //LOW or HIGH
        delayMicroseconds(10); //How long should the trigger pulse be?
        digitalWrite(trigPin, LOW); //LOW or HIGH

        // Read the signal from the sensor: a ??? pulse whose
        // duration is the time (in microseconds) from the sending
        // of the ping to the reception of its echo off of an object.
        duration = pulseIn(echoPin, HIGH, 60000); //format: pulseIn(pin, HIGH or LOW, timeout in microseconds)
        sprintf(returnMsg, "US:%d", (int)duration);
        wifi->sendPacket(returnMsg);
        break;
      }

    case 12: //---Piezo Tone---
      { tonePeriod = getPin(udpPacket); //this is the first value sent in piezoTone(period, duration)
        toneDuration = getVal(udpPacket); //this is the second value sent in piezoTone(period, duration)

        digitalWrite(2, LOW);                                //Hint: how do I ensure that this sets the M3- pin to GND?

        unsigned long toneStart = millis();                  //Hint: how do I set this variable equal to the current timestamp when this function was called?
        unsigned long elapsed_time = 0;
        while ( elapsed_time < toneDuration ) {              //Hint: how do I check whether it has been more time than the toneDuration since the toneStart?
          digitalWrite(3, HIGH);  //LOW or HIGH
          delayMicroseconds(tonePeriod / 2);                 //Hint: for a square wave with 50% duty cycle, how long should it be high/low in a given period?
          digitalWrite(3, LOW); //LOW or HIGH
          delayMicroseconds(tonePeriod / 2);                 //Hint: for a square wave with 50% duty cycle, how long should it be high/low in a given period?
          elapsed_time = millis() - toneStart;
        }
        break;
      }

    case 13:  //---set up the IR Reflectance sensor array---
      { // configure the sensors
        qtr.setTypeRC();
        const uint8_t SensorCount = 4;
        qtr.setSensorPins((const uint8_t[]) {
          7, 8, 9, 10
        }, SensorCount);

        break;
      }

    case 14: //---read IR reflectance sensor---
      { qtr.read(sensorValues);
        sprintf(returnMsg, "IR:%d:%d:%d:%d", sensorValues[0], sensorValues[1], sensorValues[2], sensorValues[3]);
        wifi->sendPacket(returnMsg);
        break;
      }

    case 15: //---Reset Encoder---
      { int enc_num = getVal(udpPacket);
        switch (enc_num) {
          case 1:
            encoder1.resetCounter(0);
            break;
          case 2:
            encoder2.resetCounter(0);
            break;
        }
        break;
      }

    case 16: //---Read Encoders (position)---
      { int enc1_cnt = encoder1.getRawCount();
        int enc2_cnt = encoder2.getRawCount();
        sprintf(returnMsg, "ENC:%d:%d", enc1_cnt, enc2_cnt);
        wifi->sendPacket(returnMsg);
        break;
      }

    case 17: //---Read Encoders (velocity)---
      { int enc1_vel = encoder1.getCountPerSecond();
        int enc2_vel = encoder2.getCountPerSecond();
        sprintf(returnMsg, "ENC_VEL:%d:%d", enc1_vel, enc2_vel);
        wifi->sendPacket(returnMsg);
        break;
      }

    case 18: //---Read RGB Sensor---
      { rgbSensor.getRGB(&red, &green, &blue);
        sprintf(returnMsg, "RGB:%d,%d,%d", int(red), int(green), int(blue));
        wifi->sendPacket(returnMsg);
        break;
      }
  }
}

// This function parses the input from UDP to extract the command number that is sent
int getCommand(String input) {
  return input.substring(0, input.indexOf(":")).toInt();
}

// This function parses the input from UDP to extract the Pin that is sent
double getPin(String input) {
  String substr = input.substring(input.indexOf(":") + 1);
  return substr.substring(0, substr.indexOf(":")).toDouble();
}

// This function parses the input from UDP to extract the value that is sent.
int getVal(String input) {
  String substr = input.substring(input.indexOf(":") + 1);
  String ssubstr = substr.substring(substr.indexOf(":") + 1);
  return ssubstr.toInt();
}



void resetReadings() {
  a1 = 0;
  a2 = 0;
  a5 = 0;
  a6 = 0;
  x = 0;
  y = 0;
  z = 0;
  sensorValues[0] = 0;
  sensorValues[1] = 0;
  sensorValues[2] = 0;
  sensorValues[3] = 0;
  duration = 0;
}
