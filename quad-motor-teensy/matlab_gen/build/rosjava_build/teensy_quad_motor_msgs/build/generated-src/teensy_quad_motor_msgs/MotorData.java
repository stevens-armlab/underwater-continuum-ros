package teensy_quad_motor_msgs;

public interface MotorData extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "teensy_quad_motor_msgs/MotorData";
  static final java.lang.String _DEFINITION = "float32 position\nfloat32 velocity\nfloat32 torque\nint32 mode\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  float getPosition();
  void setPosition(float value);
  float getVelocity();
  void setVelocity(float value);
  float getTorque();
  void setTorque(float value);
  int getMode();
  void setMode(int value);
}
