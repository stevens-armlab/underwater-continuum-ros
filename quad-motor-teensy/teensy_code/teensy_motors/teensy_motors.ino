#include <Dynamixel2Arduino.h>
#include <ros.h>
#include <teensy_quad_motor_msgs/QuadMotorData.h>
#include <std_msgs/Float32.h>

//=============================================================================================

#define DXL_SERIAL   Serial1
#define DEBUG_SERIAL Serial
const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN

const uint8_t DXL_ID1 = 201;
const uint8_t DXL_ID2 = 202;
const uint8_t DXL_ID3 = 203;
const uint8_t DXL_ID4 = 204;
const uint8_t DXL_ID_all[4] = {DXL_ID1, DXL_ID2, DXL_ID3, DXL_ID4};

const float DXL_PROTOCOL_VERSION = 2.0;
const int32_t BAUD_RATE_MOTOR = 1000000;
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

float desired_velocity[4] = {0,0,0,0};

// ROS message setup
ros::NodeHandle nh;
teensy_quad_motor_msgs::QuadMotorData motor_msg;
ros::Publisher pub_motor_pos("motor_positions", &motor_msg);


//=============================================================================================

void configureDynamixel(uint8_t motor_id){
  // Get Dynamixel information
  dxl.ping(motor_id);

  // Turn off torque when configuring items in EEPROM area
  dxl.torqueOff(motor_id);
  dxl.setOperatingMode(motor_id, OP_VELOCITY);
  dxl.torqueOn(motor_id);
}

void messageCb( const teensy_quad_motor_msgs::QuadMotorData& msg){
  for (int i=0; i<4; i++){
    desired_velocity[i] = msg.motors[i].velocity;
  }
}

ros::Subscriber<teensy_quad_motor_msgs::QuadMotorData> sub("quad_motor_command", &messageCb );


//=============================================================================================

void setup()
{
  Serial.begin(9600);
  while(!Serial);
  dxl.begin(BAUD_RATE_MOTOR);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  for (int i=0; i<4; i++){
    configureDynamixel(DXL_ID_all[i]);
    Serial.println("Dynamixel " + String(DXL_ID_all[i]) + " configured");
  }
  nh.getHardware()->setBaud(9600);
  nh.initNode();

  nh.advertise(pub_motor_pos);
  nh.subscribe(sub);
}

//=============================================================================================

void loop()
{
  for (int i=0; i<4; i++){
    motor_msg.motors[i].position = dxl.getPresentPosition(DXL_ID_all[i], UNIT_DEGREE);
    motor_msg.motors[i].velocity = dxl.getPresentVelocity(DXL_ID_all[i], UNIT_RPM);
    motor_msg.motors[i].torque = dxl.getPresentCurrent(DXL_ID_all[i], UNIT_MILLI_AMPERE);
    motor_msg.motors[i].mode = OP_POSITION;
    
    dxl.setGoalVelocity(DXL_ID_all[i], desired_velocity[i], UNIT_RPM);
  }

  pub_motor_pos.publish( &motor_msg);
  nh.spinOnce();
  delay(1);
}
