/*
 * rosserial Subscriber Example
 * Blinks an LED on callback
 */
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

const uint8_t DXL_ID = 12;
const float DXL_PROTOCOL_VERSION = 2.0;
const int32_t BAUD_RATE_MOTOR = 1000000;
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

void messageCb(const teensy_quad_motor_msgs::QuadMotorData& msg){
  dxl.setGoalPosition(DXL_ID, msg.motors[0].position, UNIT_DEGREE);
//  DEBUG_SERIAL.print(toggle_msg.data);
}

// ROS message setup
teensy_quad_motor_msgs::QuadMotorData motor_msg;
ros::Subscriber<teensy_quad_motor_msgs::QuadMotorData> sub("motor_goto_cmd", &messageCb );
ros::Publisher pub_motor_pos("motor_positions", &motor_msg);
//float motor_x = 0.0;
//float motor_test_t = 0.0;
//unsigned long motor_start_t;

void setup()
{
  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(BAUD_RATE_MOTOR);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  // Get DYNAMIXEL information
  dxl.ping(DXL_ID);

  // Turn off torque when configuring items in EEPROM area
  dxl.torqueOff(DXL_ID);
  dxl.setOperatingMode(DXL_ID, OP_POSITION);
  dxl.torqueOn(DXL_ID);
  
//  pinMode(13, OUTPUT);
  nh.initNode();
  nh.advertise(pub_motor_pos);
  nh.subscribe(sub);
}

void loop()
{
  for (int i=0; i<1; i++){
    motor_msg.motors[i].position = 
    dxl.getPresentPosition(DXL_ID, UNIT_DEGREE);
    motor_msg.motors[i].velocity = 
    dxl.getPresentVelocity(DXL_ID, UNIT_PERCENT);
    motor_msg.motors[i].mode = OP_POSITION;
    }
  pub_motor_pos.publish( &motor_msg);
  nh.spinOnce();
  delay(1);
}
