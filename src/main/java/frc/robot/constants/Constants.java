package frc.robot.constants;

/**
 * Container class for general constants.
 */
public final class Constants {
  public static final double kGravitationalAccel = 9.8;
  public static final double kRobotVoltage = 12.0;
  public static final double kLoopTime = 0.02;
  
  public static final double kCancoderResolution = 4096;

  // CAN bus names
  public static final String kCanivoreCAN = "CANivore";
  public static final String kRioCAN = "rio";

  /** The key used to access the RobotId name in the RoboRIO's persistent memory. */
  public static final String kRobotIdKey = "RobotId";

  public static final boolean kLogging = true;

  // this is not a constant...
  public static boolean kUseTelemetry = true;

  // port for the led controller, the Blinkin
  public static final int kBlinkinPort = 0;
}
