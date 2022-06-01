#include <Dynamixel2Arduino.h>
#include <ros.h>
#include <teensy_quad_motor_msgs/QuadMotorData.h>
#include <std_msgs/Float64.h>
#include <PID_v1.h>

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

//float error[4] = {0,0,0,0};
int command_received = 0;
//int potentiometer_received = 0;

// SETPOINT
double setpoint = 0;

// INPUT
double error1 = 0;
double error2 = 0;
double error3 = 0;
double error4 = 0;

// OUTPUT
double command1 = 0;
double command2 = 0;
double command3 = 0;
double command4 = 0;

// GAIN
float Kp = 20;
float Ki = 50;
float Kd = 10;

// PID
PID errorPID1(&error1, &command1, &setpoint, Kp, Ki, Kd, DIRECT);
PID errorPID2(&error2, &command2, &setpoint, Kp, Ki, Kd, DIRECT);
PID errorPID3(&error3, &command3, &setpoint, Kp, Ki, Kd, DIRECT);
PID errorPID4(&error4, &command4, &setpoint, Kp, Ki, Kd, DIRECT);

void error_cb( const teensy_quad_motor_msgs::QuadMotorData& msg){
//  for (int i=0; i<4; i++){
//    error[i] = msg.motors[i].position;
//  }
  error1 = msg.motors[1].position;
  error2 = msg.motors[2].position;
  error3 = msg.motors[3].position;
  error4 = msg.motors[4].position;
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

  errorPID1.SetMode(AUTOMATIC);
  errorPID2.SetMode(AUTOMATIC);
  errorPID3.SetMode(AUTOMATIC);
  errorPID4.SetMode(AUTOMATIC);
  errorPID1.SetOutputLimits(-1000, 1000);
  errorPID2.SetOutputLimits(-1000, 1000);
  errorPID3.SetOutputLimits(-1000, 1000);
  errorPID4.SetOutputLimits(-1000, 1000);
}

void loop()
{
  digitalWrite(13,HIGH);
  for (int i=0; i<4; i++){
    motor_msg.motors[i].position = dxl.getPresentPosition(DXL_ID_all[i], UNIT_DEGREE);
    motor_msg.motors[i].velocity = dxl.getPresentVelocity(DXL_ID_all[i], UNIT_RPM);
    motor_msg.motors[i].mode = OP_VELOCITY;
//    float command_velocity = gain*error[i];
//    if (command_received == 1)
//    {
//      dxl.setGoalVelocity(DXL_ID_all[i], command_velocity, UNIT_RAW);
//    }
  }

  errorPID1.Compute();
  errorPID2.Compute();
  errorPID3.Compute();
  errorPID4.Compute();

  dxl.setGoalVelocity(DXL_ID_all[1], command1, UNIT_RAW);
  dxl.setGoalVelocity(DXL_ID_all[2], command2, UNIT_RAW);
  dxl.setGoalVelocity(DXL_ID_all[3], command3, UNIT_RAW);
  dxl.setGoalVelocity(DXL_ID_all[4], command4, UNIT_RAW);

  pub_motor_pos.publish( &motor_msg);
  nh.spinOnce();
  delay(1);
}
