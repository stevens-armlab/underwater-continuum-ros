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

const uint8_t DXL_ID1 = 101;
const uint8_t DXL_ID2 = 102;
const uint8_t DXL_ID3 = 103;
const uint8_t DXL_ID4 = 104;
const float DXL_PROTOCOL_VERSION = 2.0;
const int32_t BAUD_RATE_MOTOR = 1000000;
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

void setup() {
  // put your setup code here, to run once:
  
  // Use UART port of DYNAMIXEL Shield to debug.
  DEBUG_SERIAL.begin(115200);

  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(BAUD_RATE_MOTOR);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  // Get DYNAMIXEL information
  dxl.ping(DXL_ID1);
  dxl.ping(DXL_ID2);
  dxl.ping(DXL_ID3);
  dxl.ping(DXL_ID4);

  // Turn off torque when configuring items in EEPROM area
  dxl.torqueOff(DXL_ID1);
  dxl.setOperatingMode(DXL_ID1, OP_POSITION);
  dxl.torqueOn(DXL_ID1);

  dxl.torqueOff(DXL_ID2);
  dxl.setOperatingMode(DXL_ID2, OP_POSITION);
  dxl.torqueOn(DXL_ID2);

  dxl.torqueOff(DXL_ID3);
  dxl.setOperatingMode(DXL_ID3, OP_POSITION);
  dxl.torqueOn(DXL_ID3);

  dxl.torqueOff(DXL_ID4);
  dxl.setOperatingMode(DXL_ID4, OP_POSITION);
  dxl.torqueOn(DXL_ID4);
}

void loop() {
  // put your main code here, to run repeatedly:
  
  // Please refer to e-Manual(http://emanual.robotis.com/docs/en/parts/int512erface/dynamixel_shield/) for available range of value. 
  // Set Goal Position in RAW value512
  dxl.setGoalPosition(DXL_ID1, 0, UNIT_DEGREE);
  dxl.setGoalPosition(DXL_ID2, 0, UNIT_DEGREE);
  dxl.setGoalPosition(DXL_ID3, 0, UNIT_DEGREE);
  dxl.setGoalPosition(DXL_ID4, 0, UNIT_DEGREE);
  delay(2000);
  // Print present position in raw value
  DEBUG_SERIAL.print("Present Position(degree) : ");
  DEBUG_SERIAL.println(dxl.getPresentPosition(DXL_ID1, UNIT_DEGREE));
  DEBUG_SERIAL.println(dxl.getPresentPosition(DXL_ID2, UNIT_DEGREE));
  DEBUG_SERIAL.println(dxl.getPresentPosition(DXL_ID3, UNIT_DEGREE));
  DEBUG_SERIAL.println(dxl.getPresentPosition(DXL_ID4, UNIT_DEGREE));
  delay(2000);

  // Set Goal Position in DEGREE value
  dxl.setGoalPosition(DXL_ID1, 360, UNIT_DEGREE);
  dxl.setGoalPosition(DXL_ID2, 360, UNIT_DEGREE);
  dxl.setGoalPosition(DXL_ID3, 360, UNIT_DEGREE);
  dxl.setGoalPosition(DXL_ID4, 360, UNIT_DEGREE);
  delay(2000);
  // Print present position in degree value
  DEBUG_SERIAL.print("Present Position(degree) : ");
  DEBUG_SERIAL.println(dxl.getPresentPosition(DXL_ID1, UNIT_DEGREE));
  DEBUG_SERIAL.println(dxl.getPresentPosition(DXL_ID2, UNIT_DEGREE));
  DEBUG_SERIAL.println(dxl.getPresentPosition(DXL_ID3, UNIT_DEGREE));
  DEBUG_SERIAL.println(dxl.getPresentPosition(DXL_ID4, UNIT_DEGREE));
  delay(2000);
}
