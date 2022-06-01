
#include <Dynamixel2Arduino.h>
#include <ros.h>
#include <teensy_quad_motor_msgs/QuadMotorData.h>
#include <std_msgs/Float64.h>

ros::NodeHandle nh;

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
const uint8_t DXL_ID_all[4] = {DXL_ID1, DXL_ID2, DXL_ID3, DXL_ID4};

const float DXL_PROTOCOL_VERSION = 2.0;
const int32_t BAUD_RATE_MOTOR = 1000000;
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

float error[4] = {0,0,0,0};
int command_received = 0;
//int potentiometer_received = 0;

float gain = 60;
//float start_pos = {0, 0, 0, 0}

void error_cb( const teensy_quad_motor_msgs::QuadMotorData& msg){
  for (int i=0; i<4; i++){
    error[i] = msg.motors[i].position;
  }
  command_received = 1;
}

//void potentiometer_cb( const teensy_quad_motor_msgs::potData& msg){
//  pot_volt[1] = msg.Pot1;
//  pot_volt[2] = msg.Pot2;
//  pot_volt[3] = msg.Pot3;
//  pot_volt[4] = msg.Pot4;
//  potentiometer_received = 1;
//}

// ROS message setup
teensy_quad_motor_msgs::QuadMotorData motor_msg;
ros::Subscriber<teensy_quad_motor_msgs::QuadMotorData> sub("quad_motor_command", &error_cb );
//ros::Subscriber<teensy_quad_motor_msgs::potData> sub("pubpot", &potentiometer_received );
ros::Publisher pub_motor_pos("motor_positions", &motor_msg);

void configureDynamixel(uint8_t motor_id){
  // Get Dynamixel information
  dxl.ping(motor_id);

  // Turn off torque when configuring items in EEPROM area
  dxl.torqueOff(motor_id);
  dxl.setOperatingMode(motor_id, OP_VELOCITY);
  dxl.torqueOn(motor_id);
}

void setup()
{
  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(BAUD_RATE_MOTOR);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  // Get DYNAMIXEL information

  for (int i=0; i<4; i++){
    configureDynamixel(DXL_ID_all[i]);
  }
  
  nh.initNode();
  nh.advertise(pub_motor_pos);
  nh.subscribe(sub);

  pinMode(13, OUTPUT);
}

void loop()
{
  digitalWrite(13,HIGH);
  for (int i=0; i<4; i++){
    motor_msg.motors[i].position = dxl.getPresentPosition(DXL_ID_all[i], UNIT_DEGREE);
    motor_msg.motors[i].velocity = dxl.getPresentVelocity(DXL_ID_all[i], UNIT_RPM);
    motor_msg.motors[i].mode = OP_VELOCITY;

    float command_velocity = gain*error[i];
    if (command_received == 1)
    {
      dxl.setGoalVelocity(DXL_ID_all[i], command_velocity, UNIT_RAW);
    }
  }
    
  pub_motor_pos.publish( &motor_msg);
  nh.spinOnce();
  delay(1);
}
