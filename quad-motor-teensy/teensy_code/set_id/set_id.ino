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

const uint8_t DXL_ID = 1;
const uint8_t DXL_ID_NEW = 12;
const float DXL_PROTOCOL_VERSION = 2.0;
const int32_t BAUD_RATE_MOTOR = 1000000;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

void setup() {
  // put your setup code here, to run once:
  
  // Use UART port of DYNAMIXEL Shield to debug.
  DEBUG_SERIAL.begin(115200);
  while(!DEBUG_SERIAL);
  DEBUG_SERIAL.println("debug serial setup");
  
  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(BAUD_RATE_MOTOR);
  DEBUG_SERIAL.println("motor dxl serial startted");
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  DEBUG_SERIAL.print("PROTOCOL ");
  DEBUG_SERIAL.print(DXL_PROTOCOL_VERSION, 1);
  DEBUG_SERIAL.print(", ID ");
  DEBUG_SERIAL.print(DXL_ID);
  DEBUG_SERIAL.print(": ");
  if(dxl.ping(DXL_ID) == true) {
    DEBUG_SERIAL.print("ping succeeded!");
    DEBUG_SERIAL.print(", Model Number: ");
    DEBUG_SERIAL.println(dxl.getModelNumber(DXL_ID));
    
    // Turn off torque when configuring items in EEPROM area
    dxl.torqueOff(DXL_ID);
    
    // set a new ID for DYNAMIXEL. Do not use ID 200
    dxl.setID(DXL_ID, DXL_ID_NEW);
    DEBUG_SERIAL.print("ID has been successfully changed to ");
    DEBUG_SERIAL.print(DXL_ID_NEW);
  }
  else{
    DEBUG_SERIAL.println("ping failed!");
  }
}

void loop() {
  // put your main code here, to run repeatedly:
}
