#include <Wire.h>
#include <SparkFun_I2C_Mux_Arduino_Library.h>
#include "SparkFun_BNO080_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_BNO080

#include <ros.h>
#include <potentiometer_data/potData.h>
#include <imu_data/imu_angles.h>
#include <std_msgs/Float32.h>

//=============================================================================================


#define NUMBER_OF_SENSORS 8
BNO080 myIMU;
QWIICMUX myMux;

int sensorPin0 = A10; //input pin for the first pot
int sensorPin1 = A11; //input pin for the second pot
int sensorPin2 = A12; //input pin for the third pot
int sensorPin3 = A13; //input pin for the fourth pot

float averageVolt0 = 0;
float averageVolt1 = 0;
float averageVolt2 = 0;
float averageVolt3 = 0;
#define AVERAGE_WINDOW 10
float volt0[AVERAGE_WINDOW] = {0};
float volt1[AVERAGE_WINDOW] = {0};
float volt2[AVERAGE_WINDOW] = {0};
float volt3[AVERAGE_WINDOW] = {0};
int led=13;

// ROS message setup
ros::NodeHandle nh;
imu_data::imu_angles imu;
potentiometer_data::potData pd_msg;
//ros::Publisher pubimu("pubimu", &imu);
ros::Publisher pubpot("pubpot", &pd_msg);

//=============================================================================================

void ReadPot() {
  float sum0 = 0;
  float sum1 = 0;
  float sum2 = 0;
  float sum3 = 0;
  for(int i=AVERAGE_WINDOW-1; i>=0; i--){
    
    if (i==0){
      volt0[i] = analogRead(sensorPin0) * (3.3/1023);
      volt1[i] = analogRead(sensorPin1) * (3.3/1023);
      volt2[i] = analogRead(sensorPin2) * (3.3/1023);
      volt3[i] = analogRead(sensorPin3) * (3.3/1023);
    }
    
    else{
      volt0[i] = volt0[i-1];
      volt1[i] = volt1[i-1];
      volt2[i] = volt2[i-1];
      volt3[i] = volt3[i-1];
    }
    sum0 = sum0 + volt0[i];
    sum1 = sum1 + volt1[i];
    sum2 = sum2 + volt2[i];
    sum3 = sum3 + volt3[i];
  }


  averageVolt0 = sum0/AVERAGE_WINDOW;
  averageVolt1 = sum1/AVERAGE_WINDOW;
  averageVolt2 = sum2/AVERAGE_WINDOW;
  averageVolt3 = sum3/AVERAGE_WINDOW;

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
}

//=============================================================================================

void setup()
{
  pinMode(led,OUTPUT);
  Serial.begin(9600);
  while(!Serial);
  nh.getHardware()->setBaud(9600);
  nh.initNode();

//  Wire.begin();
//
//  if (myMux.begin() == false) {
//    Serial.println("Mux not detected. Freezing...");
//    delay(1);
//    while (1);
//  }
//   
//  Serial.println("Mux detected");
//  for (byte x = 0; x < NUMBER_OF_SENSORS; x++) {
//    delay(50);
//    myMux.enablePort(x);
//   delay(50);
//    if(myIMU.begin(0x4A) == false) {
//      Serial.print("Sensor ");
//      Serial.print(x);
//      Serial.println(" did not begin! Retrying...");
//      x--;   //If sensor x doesn't start, keep on trying until it does
//    }
//   
//    else {
//      Serial.print("Sensor ");
//      Serial.print(x);
//      Serial.println(" configured!");
//    }
//   
//    Wire.setClock(400000); //Increase I2C data rate to 400kH
//    myIMU.enableRotationVector(2); //Send data update every 50ms
//    myMux.disablePort(x);
//  }
 
  Serial.println(F("Rotation vector enabled"));
  Serial.println(F("IMU initiation complete"));
//  nh.advertise(pubimu);
  nh.advertise(pubpot);
}

//=============================================================================================

void loop()
{
//  for (byte x = 0; x < NUMBER_OF_SENSORS; x++) {
//    delay(1);
//    myMux.enablePort(x);
//
//    delay(1);
//    if (myIMU.dataAvailable() == true) {
//      publishImu(x);
//    }
//    
//    delay(1);
//    myMux.disablePort(x);
//  }

digitalWrite(led, HIGH);
  
  ReadPot();

  pd_msg.Pot1 = averageVolt0;
  pd_msg.Pot2 = averageVolt1;
  pd_msg.Pot3 = averageVolt2;
  pd_msg.Pot4 = averageVolt3;

  pubpot.publish(&pd_msg);
//  pubimu.publish( &imu );
  nh.spinOnce();
  delay(5);
}
