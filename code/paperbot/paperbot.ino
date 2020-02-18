/*
  Wireless Servo Control, with ESP as Access Point

  Usage:
    Connect phone or laptop to "ESP_XXXX" wireless network, where XXXX is the ID of the robot
    Go to 192.168.4.1.
    A webpage with four buttons should appear. Click them to move the robot.

  Installation:
    In Arduino, go to Tools > ESP8266 Sketch Data Upload to upload the files from ./data to the ESP
    Then, in Arduino, compile and upload sketch to the ESP

  Requirements:
    Arduino support for ESP8266 board
      In Arduino, add URL to Files > Preferences > Additional Board Managers URL.
      See https://learn.sparkfun.com/tutorials/esp8266-thing-hookup-guide/installing-the-esp8266-arduino-addon

    Websockets library
      To install, Sketch > Include Library > Manage Libraries... > Websockets > Install
      https://github.com/Links2004/arduinoWebSockets

    ESP8266FS tool
      To install, create "tools" folder in Arduino, download, and unzip. See
      https://github.com/esp8266/Arduino/blob/master/doc/filesystem.md#uploading-files-to-file-system

  Hardware:
    NodeMCU Amica DevKit Board (ESP8266 chip)
    Motorshield for NodeMCU
    2 continuous rotation servos plugged into motorshield pins D1, D2
    Ultra-thin power bank
    Paper chassis

*/

#include <Arduino.h>

#include <Hash.h>
#include <FS.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <WebSocketsServer.h>
#include <ESP8266mDNS.h>

#include <Servo.h>
#include <Wire.h>
#include <VL53L0X.h>

#include "debug.h"
#include "file.h"
#include "server.h"

#include "MPU9250.h"

#define SDA_PORT D14
#define SCL_PORT D12
VL53L0X sensor;
VL53L0X sensor2;

const int SERVO_LEFT = D1;
const int SERVO_RIGHT = D2;
Servo servo_left;
Servo servo_right;
int servo_left_ctr = 90;
int servo_right_ctr = 90;


// WiFi AP parameters
char ap_ssid[13];
char* ap_password = "";

// WiFi STA parameters
char* sta_ssid =
  "...";
char* sta_password =
  "...";

char* mDNS_name = "paperbot";

String html;
String css;

MPU9250 IMU(Wire, 0x68);
int status;

void lidar_setup() {
  pinMode(D3, OUTPUT);
  pinMode(D4, OUTPUT);
  digitalWrite(D7, LOW);
  digitalWrite(D8, LOW);

  delay(500);
  Wire.begin(SDA_PORT, SCL_PORT);

  Serial.begin (115200);

  digitalWrite(D3, HIGH);
  delay(150);
  Serial.println("00");

  sensor.init(true);
  Serial.println("01");
  delay(100);
  sensor.setAddress((uint8_t)22);

  digitalWrite(D4, HIGH);
  delay(150);
  sensor2.init(true);
  Serial.println("03");
  delay(100);
  sensor2.setAddress((uint8_t)25);
  Serial.println("04");

  Serial.println("addresses set");

  Serial.println ("I2C scanner. Scanning ...");
  byte count = 0;

  for (byte i = 1; i < 120; i++)
  {

    Wire.beginTransmission (i);
    if (Wire.endTransmission () == 0)
    {
      Serial.print ("Found address: ");
      Serial.print (i, DEC);
      Serial.print (" (0x");
      Serial.print (i, HEX);
      Serial.println (")");
      count++;
      delay (1);  // maybe unneeded?
    } // end of good response
  } // end of for loop
  Serial.println ("Done.");
  Serial.print ("Found ");
  Serial.print (count, DEC);
  Serial.println (" device(s).");

}

void setup() {
  setupPins();

  sprintf(ap_ssid, "ESP_%08X", ESP.getChipId());

  for (uint8_t t = 4; t > 0; t--) {
    Serial.printf("[SETUP] BOOT WAIT %d...\n", t);
    Serial.flush();
    LED_ON;
    delay(500);
    LED_OFF;
    delay(500);
  }
  LED_ON;
  //setupSTA(sta_ssid, sta_password);
  setupAP(ap_ssid, ap_password);
  LED_OFF;

  setupFile();
  html = loadFile("/controls.html");
  css = loadFile("/style.css");
  registerPage("/", "text/html", html);
  registerPage("/style.css", "text/css", css);

  setupHTTP();
  setupWS(webSocketEvent);
  //setupMDNS(mDNS_name);

  // serial to display data
  Serial.begin(115200);
  while (!Serial) {}

  // start communication with IMU
  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while (1) {}
  }

  lidar_setup();

  stop();

}

//Testing the gyroscope sensor
//For various PWM levels, the output of the gyro is compared to what is measured in real life
void loop() {
  wsLoop();
  httpLoop();

  /* Testing for simulation 
     Uncommented while testing for sensor model */ 

  //straight(); 
  //turn_right(); 

  /* Testing for Sensor Model */
  int i;
  //loop to drive car at various PWM
  for (i = -10; i <= 200; i += 10) {
    drive(i, i);
    float gyroValue = 0;
    float angle = 0;
    float gyroRate;
    //Loop to capture theta from theta dot of the gyro
    int j;
    for (j = 0; j < 1000; j++) {
      IMU.readSensor();
      gyroRate = IMU.getGyroZ_rads(); //car turns around Z axis
      gyroValue = gyroRate * (180 / 3.14); //take the speed in rad and convert to degrees
      angle += gyroValue / 100.0; //theta = theta_intial + (theta_dot/dt); theta_inital = 0

      //format of arduino output for ppython parser is:
      //lidar1, lidar2, theta dot, theta
      Serial.print(sensor.readRangeSingleMillimeters());
      if (sensor.timeoutOccurred()) {
        Serial.print(" TIMEOUT");
      }
      Serial.print(,);
      Serial.print(sensor2.readRangeSingleMillimeters());
      if (sensor.timeoutOccurred()) {
        Serial.print(" TIMEOUT");
      }
      Serial.print(,);
      Serial.print(gyroRate);
      Serial.print(",");
      Serial.println(angle );

      delay(10);
    }
  }
  stop();
  delay(10000);
}

//-----Running car for simulation-----//

/*
  Testing driving the car forward until it reaches a wall
*/
void straight() {
  float front = 0;
  float right = 0;

  //if there is room in the front of the car 
  while (front >= 30) {
    forward(); //drive the car forward 

    //read in the gyroscope data 
    float gyroValue = 0;
    float angle = 0;
    float gyroRate;
    IMU.readSensor();
    gyroRate = IMU.getGyroZ_rads(); //car turns around Z axis
    gyroValue = gyroRate * (180 / 3.14); //take the speed in rad and convert to degrees
    angle += gyroValue / 100.0; //theta = theta_intial + (theta_dot/dt); theta_inital = 0

    //format of arduino output for ppython parser is:
    //lidar1, lidar2, theta dot, theta
    front = sensor.readRangeSingleMillimeters();
    Serial.print(front);
    if (sensor.timeoutOccurred()) {
      Serial.print(" TIMEOUT");
    }
    Serial.print(,);
    right = sensor2.readRangeSingleMillimeters();
    Serial.print(right);
    if (sensor.timeoutOccurred()) {
      Serial.print(" TIMEOUT");
    }
    Serial.print(,);
    Serial.print(gyroRate);
    Serial.print(",");
    Serial.println(angle);

    delay(10); 

  }

  //when car is close to wall stop and end simulation run 
  stop();
  delay(500);
}

/*
  Testing the car's ability to turn right
*/
void turn_right() {

  //same as straight -> drive forward till reach wall 
  float front = 0;
  float right = 0;
  while (front >= 30) {
    forward();
    float gyroValue = 0;
    float angle = 0;
    float gyroRate;

    IMU.readSensor();
    gyroRate = IMU.getGyroZ_rads(); //car turns around Z axis
    gyroValue = gyroRate * (180 / 3.14); //take the speed in rad and convert to degrees
    angle += gyroValue / 100.0; //theta = theta_intial + (theta_dot/dt); theta_inital = 0

    //format of arduino output for ppython parser is:
    //lidar1, lidar2, theta dot, theta
    front = sensor.readRangeSingleMillimeters();
    Serial.print(front);
    if (sensor.timeoutOccurred()) {
      Serial.print(" TIMEOUT");
    }
    Serial.print(,);
    right = sensor2.readRangeSingleMillimeters();
    Serial.print(right);
    if (sensor.timeoutOccurred()) {
      Serial.print(" TIMEOUT");
    }
    Serial.print(,);
    Serial.print(gyroRate);
    Serial.print(",");
    Serial.println(angle);

    delay(10);

  }

  //check to see if there is space to the right 
  if (right >= 30) {
    right(); //if there is turn right 

    //use gyroscope to know when fully turned to the right by 90 degrees 
    //assume the car moves forward at 90 degrees initially 
    while (!((angle > -10) && (angle < 10))) {
      IMU.readSensor();
      gyroRate = IMU.getGyroZ_rads(); //car turns around Z axis
      gyroValue = gyroRate * (180 / 3.14); //take the speed in rad and convert to degrees
      angle += gyroValue / 100.0; //theta = theta_intial + (theta_dot/dt); theta_inital = 0
      delay(10);
    }
    stop(); 
  }
  stop();
  delay(500);
}


//
// Movement Functions //
//

void drive(int left, int right) {
  servo_left.write(left);
  servo_right.write(right);
}

void stop() {
  DEBUG("stop");
  drive(servo_left_ctr, servo_right_ctr);
  LED_OFF;
}

void forward() {
  DEBUG("forward");
  drive(0, 180);
}

void backward() {
  DEBUG("backward");
  drive(180, 0);
}

void left() {
  DEBUG("left");
  drive(180, 180);
}

void right() {
  DEBUG("right");
  drive(0, 0);
}



//
// Setup //
//

void setupPins() {
  // setup Serial, LEDs and Motors
  Serial.begin(115200);
  DEBUG("Started serial.");

  pinMode(LED_PIN, OUTPUT);    //Pin D0 is LED
  LED_OFF;                     //Turn off LED
  DEBUG("Setup LED pin.");

  servo_left.attach(SERVO_LEFT);
  servo_right.attach(SERVO_RIGHT);
  DEBUG("Setup motor pins");
}

void webSocketEvent(uint8_t id, WStype_t type, uint8_t * payload, size_t length) {

  switch (type) {
    case WStype_DISCONNECTED:
      DEBUG("Web socket disconnected, id = ", id);
      break;
    case WStype_CONNECTED:
      {
        // IPAddress ip = webSocket.remoteIP(id);
        // Serial.printf("[%u] Connected from %d.%d.%d.%d url: %s\n", id, ip[0], ip[1], ip[2], ip[3], payload);
        DEBUG("Web socket connected, id = ", id);

        // send message to client
        wsSend(id, "Connected to ");
        wsSend(id, ap_ssid);
        break;
      }
    case WStype_BIN:
      DEBUG("On connection #", id)
      DEBUG("  got binary of length ", length);
      for (int i = 0; i < length; i++)
        DEBUG("    char : ", payload[i]);

      if (payload[0] == '~')
        drive(180 - payload[1], payload[2]);

    case WStype_TEXT:
      DEBUG("On connection #", id)
      DEBUG("  got text: ", (char *)payload);

      if (payload[0] == '#') {
        if (payload[1] == 'C') {
          LED_ON;
          wsSend(id, "Hello world!");
        }
        else if (payload[1] == 'F')
          forward();
        else if (payload[1] == 'B')
          backward();
        else if (payload[1] == 'L')
          left();
        else if (payload[1] == 'R')
          right();
        else if (payload[1] == 'U') {
          if (payload[2] == 'L')
            servo_left_ctr -= 1;
          else if (payload[2] == 'R')
            servo_right_ctr += 1;
          char tx[20] = "Zero @ (xxx, xxx)";
          sprintf(tx, "Zero @ (%3d, %3d)", servo_left_ctr, servo_right_ctr);
          wsSend(id, tx);
        }
        else if (payload[1] == 'D') {
          if (payload[2] == 'L')
            servo_left_ctr += 1;
          else if (payload[2] == 'R')
            servo_right_ctr -= 1;
          char tx[20] = "Zero @ (xxx, xxx)";
          sprintf(tx, "Zero @ (%3d, %3d)", servo_left_ctr, servo_right_ctr);
          wsSend(id, tx);
        }
        else
          stop();
      }

      break;
  }
}
