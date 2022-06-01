#include <Wire.h>
#include <SparkFun_I2C_Mux_Arduino_Library.h>
#include <ros.h>
#include <std_msgs/Float32.h>
#include <potentiometer_data/potData.h>
#include <imu_data/imu_angles.h>
#include "SparkFun_BNO080_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_BNO080

BNO080 myIMU;

QWIICMUX myMux;

#define NUMBER_OF_SENSORS 8

ros::NodeHandle  nh;
imu_data::imu_angles imu;
potentiometer_data::potData pd_msg;
ros::Publisher pubimu("pubimu", &imu);
ros::Publisher pubpot("pubpot", &pd_msg);

int sensorPin0 = A10; //input pin for the first pot
int sensorPin1 = A11; //input pin for the second pot
int sensorPin2 = A12; //input pin for the third pot
int sensorPin3 = A13; //input pin for the fourth pot

float averageVoltageValue0;
float averageVoltageValue1;
float averageVoltageValue2;
float averageVoltageValue3;

void ReadPot0() {
  Serial.begin(9600);
  float voltageValue = 0; //voltage value coming from the pot
  float netVoltageValue = 0; //sum of the 50 inputs, to be averaged
 
  for(int i = 0; i<50; i++){
    voltageValue = analogRead(sensorPin0) * (3.3/1023);
    netVoltageValue = netVoltageValue + voltageValue;
    }

 averageVoltageValue0 = netVoltageValue / 50;
  //Serial.println(averageVoltageValue0);
  }

void ReadPot1() {
  float voltageValue = 0; //voltage value coming from the pot
  float netVoltageValue = 0; //sum of the 50 inputs, to be averaged
 
  for(int i = 0; i<50; i++) {
    voltageValue = analogRead(sensorPin1) * (3.3/1023);
    netVoltageValue = netVoltageValue + voltageValue;
    }

  averageVoltageValue1 = netVoltageValue / 50;
  }

void ReadPot2() {
  float voltageValue = 0; //voltage value coming from the pot
  float netVoltageValue = 0; //sum of the 50 inputs, to be averaged
 
  for(int i = 0; i<50; i++){
    voltageValue = analogRead(sensorPin2) * (3.3/1023);
    netVoltageValue = netVoltageValue + voltageValue;
    }

  averageVoltageValue2 = netVoltageValue / 50;
  }

void ReadPot3() {
  float voltageValue = 0; //voltage value coming from the pot
  float netVoltageValue = 0; //sum of the 50 inputs, to be averaged
 
  for(int i = 0; i<50; i++){
    voltageValue = analogRead(sensorPin3) * (3.3/1023);
    netVoltageValue = netVoltageValue + voltageValue;
    }

  averageVoltageValue3 = netVoltageValue / 50;
  }

void setup() {
  nh.getHardware()->setBaud(9600);
  nh.initNode();
  Serial.begin(9600);
  while(!Serial);
  Serial.println();
  Serial.println("BNO080 Read Example");

  Wire.begin();

    if (myMux.begin() == false) {
    Serial.println("Mux not detected. Freezing...");
    while (1)
      ;
    }
   
  Serial.println("Mux detected");
  for (byte x = 0; x < NUMBER_OF_SENSORS; x++) {
    delay(50);
    myMux.enablePort(x);
   
    if(myIMU.begin(0x4A) == false) {
      Serial.print("Sensor ");
      Serial.print(x);
      Serial.println(" did not begin! Retrying...");
      x--;   //If sensor x doesn't start, keep on trying until it does
    }
   
    else {
      Serial.print("Sensor ");
      Serial.print(x);
      Serial.println(" configured!");
    }
   
    Wire.setClock(400000); //Increase I2C data rate to 400kH
    myIMU.enableRotationVector(50); //Send data update every 50ms
    myMux.disablePort(x);
    }
 
  Serial.println(F("Rotation vector enabled"));
  Serial.println(F("IMU initiation complete"));
  nh.advertise(pubimu);
  nh.advertise(pubpot);
}

void publishImu(byte x) {
  if (x == 0) {
    imu.x0 = myIMU.getQuatI();
    imu.y0 = myIMU.getQuatJ();
    imu.z0 = myIMU.getQuatK();
    imu.w0 = myIMU.getQuatReal();
  }
  if (x == 1) {
    imu.x1 = myIMU.getQuatI();
    imu.y1 = myIMU.getQuatJ();
    imu.z1 = myIMU.getQuatK();
    imu.w1 = myIMU.getQuatReal();
  }
  if (x == 2) {
    imu.x2 = myIMU.getQuatI();
    imu.y2 = myIMU.getQuatJ();
    imu.z2 = myIMU.getQuatK();
    imu.w2 = myIMU.getQuatReal();
  }
  if (x == 3) {
    imu.x3 = myIMU.getQuatI();
    imu.y3 = myIMU.getQuatJ();
    imu.z3 = myIMU.getQuatK();
    imu.w3 = myIMU.getQuatReal();
  }
  if (x == 4) {
    imu.x4 = myIMU.getQuatI();
    imu.y4 = myIMU.getQuatJ();
    imu.z4 = myIMU.getQuatK();
    imu.w4 = myIMU.getQuatReal();
  }
  if (x == 5) {
    imu.x5 = myIMU.getQuatI();
    imu.y5 = myIMU.getQuatJ();
    imu.z5 = myIMU.getQuatK();
    imu.w5 = myIMU.getQuatReal();
  }
  if (x == 6) {
    imu.x6 = myIMU.getQuatI();
    imu.y6 = myIMU.getQuatJ();
    imu.z6 = myIMU.getQuatK();
    imu.w6 = myIMU.getQuatReal();
  }
  if (x == 7) {
    imu.x7 = myIMU.getQuatI();
    imu.y7 = myIMU.getQuatJ();
    imu.z7 = myIMU.getQuatK();
    imu.w7 = myIMU.getQuatReal();
  }
    pubimu.publish( &imu );
    nh.spinOnce();
}

void loop() {
  for (byte x = 0; x < NUMBER_OF_SENSORS; x++) {
    delay(10);
    myMux.enablePort(x);
    Serial.print("CurrentPort: ");
    Serial.println(x);

    if (myIMU.dataAvailable() == true) {
    //imu.x0 = myIMU.getQuatI();
    //imu.y0 = myIMU.getQuatJ();
    //imu.z0 = myIMU.getQuatK();
    //imu.w0 = myIMU.getQuatReal();

    publishImu(x);
   
   
    Serial.print(imu.x0, 2);
    Serial.print(F(","));
    Serial.print(imu.y0, 2);
    Serial.print(F(","));
    Serial.print(imu.z0, 2);
    Serial.print(F(","));
    Serial.print(imu.w0, 2);
    Serial.println();
   
    Serial.println();
    Serial.println();
    }
    myMux.disablePort(x);
  }
 
  Serial.begin(9600);
 
  ReadPot0();
  ReadPot1();
  ReadPot2();
  ReadPot3();
  Serial.println(averageVoltageValue0);

  float pub0 = averageVoltageValue0;
  float pub1 = averageVoltageValue1;
  float pub2 = averageVoltageValue2;
  float pub3 = averageVoltageValue3;
 
  pd_msg.Pot1 = pub0;
  pd_msg.Pot2 = pub1;
  pd_msg.Pot3 = pub2;
  pd_msg.Pot4 = pub3;

  pubpot.publish(&pd_msg);
  nh.spinOnce();
}
