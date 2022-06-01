package teensy_quad_motor_msgs;

public interface QuadMotorData extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "teensy_quad_motor_msgs/QuadMotorData";
  static final java.lang.String _DEFINITION = "MotorData[4] motors";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  java.util.List<teensy_quad_motor_msgs.MotorData> getMotors();
  void setMotors(java.util.List<teensy_quad_motor_msgs.MotorData> value);
}
