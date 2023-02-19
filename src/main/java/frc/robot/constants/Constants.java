package frc.robot.constants;

/**
 * Container class for general constants.
 */
public final class Constants {
  public static final double kGravitationalAccel = 9.8;
  public static final double kMaxVoltage = 12.0;
  public static final double kLoopTime = 0.02;
  
  public static final double kCancoderResolution = 4096;

  // CAN bus names
  public static final String kCanivoreCAN = "CANivore";
  public static final String kRioCAN = "rio";

  // RobotId key in rio preferences
  public static final String kRobotIdKey = "RobotId";
}
