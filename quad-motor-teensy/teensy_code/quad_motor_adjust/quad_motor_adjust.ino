/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

#include <Dynamixel2Arduino.h>

#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
  #define DXL_SERIAL   Serial
  #define DEBUG_SERIAL soft_serial
  const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#else
  #define DXL_SERIAL   Serial1
  #define DEBUG_SERIAL Serial
  const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#endif

// Defined four motors with IDs 101-104 and 1M baudrate, using set_id and set_baudrate
const uint8_t DXL_ID1 = 101;
const uint8_t DXL_ID2 = 102;
const uint8_t DXL_ID3 = 103;
const uint8_t DXL_ID4 = 104;
const float DXL_PROTOCOL_VERSION = 2.0;
const int32_t BAUD_RATE_MOTOR = 1000000;
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

char input;
boolean newData = false;

void setup() {
  // put your setup code here, to run once:
  
  // Use UART port of DYNAMIXEL Shield to debug.
  DEBUG_SERIAL.begin(115200);

  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(BAUD_RATE_MOTOR);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  // Created function to configure Dynamixel for velocity mode
  configureDynamixel(DXL_ID1);
  configureDynamixel(DXL_ID2);
  configureDynamixel(DXL_ID3);
  configureDynamixel(DXL_ID4);
  
}

void loop() {
  // put your main code here, to run repeatedly:

  // readSerial function uses Serial input to run the following switch case
  readSerial();
  if(newData == true){
    switch(input){
      case 'q':
        Serial.println("Input = q, increasing motor 1 position");
        dxl.setGoalVelocity(DXL_ID1, -10, UNIT_PERCENT);
        delay(100);
        newData = false;
        break;
      
      case 'a':
        Serial.println("Input = a, decreasing motor 1 position");
        dxl.setGoalVelocity(DXL_ID1, 10, UNIT_PERCENT);
        delay(100);
        newData = false;
        break;
        
      case 'w':
        Serial.println("Input = w, increasing motor 2 position");
        dxl.setGoalVelocity(DXL_ID2, -10, UNIT_PERCENT);
        delay(100);
        newData = false;
        break;

      case 's':
        Serial.println("Input = s, decreasing motor 2 position");
        dxl.setGoalVelocity(DXL_ID2, 10, UNIT_PERCENT);
        delay(100);
        newData = false;
        break;

      case 'e':
        Serial.println("Input = e, increasing motor 3 position");
        dxl.setGoalVelocity(DXL_ID3, -10, UNIT_PERCENT);
        delay(100);
        newData = false;
        break;

      case 'd':
        Serial.println("Input = d, decreasing motor 3 position");
        dxl.setGoalVelocity(DXL_ID3, 10, UNIT_PERCENT);
        delay(100);
        newData = false;
        break;

      case 'r':
        Serial.println("Input = r, increasing motor 4 position");
        dxl.setGoalVelocity(DXL_ID4, -10, UNIT_PERCENT);
        delay(100);
        newData = false;
        break;

      case 'f':
        Serial.println("Input = f, decreasing motor 4 position");
        dxl.setGoalVelocity(DXL_ID4, 10, UNIT_PERCENT);
        delay(100);
        newData = false;
        break;

      // Serial communication "double reads" the enter key incorrectly, so it is ignored in the switch case
      case 10:
        break;

      // Enter any other key to stop the motors
      default:
        Serial.println("Stopping all motors");
        dxl.setGoalVelocity(DXL_ID1, 0, UNIT_PERCENT);
        dxl.setGoalVelocity(DXL_ID2, 0, UNIT_PERCENT);
        dxl.setGoalVelocity(DXL_ID3, 0, UNIT_PERCENT);
        dxl.setGoalVelocity(DXL_ID4, 0, UNIT_PERCENT);
        delay(100);
        newData = false;
        break;
    }
  }
}

void readSerial() {
  if(Serial.available() > 0){
    input = Serial.read();
    newData = true;
  }
}

// function to configure a new motor, for convenience
void configureDynamixel(uint8_t motor_id){
  // Get Dynamixel information
  dxl.ping(motor_id);

  // Turn off torque when configuring items in EEPROM area
  dxl.torqueOff(motor_id);
  dxl.setOperatingMode(motor_id, OP_VELOCITY);
  dxl.torqueOn(motor_id);
}
