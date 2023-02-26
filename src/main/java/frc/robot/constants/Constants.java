package frc.robot.constants;

import edu.wpi.first.math.util.Units;

/**
 * Container class for general constants.
 */
public final class Constants {

  public static final double kRobotLengthWithBumpers = Units.inchesToMeters(26 + 3.75 * 2);

  public static final double kGravitationalAccel = 9.8;
  public static final double kMaxVoltage = 12.0;
  public static final double kLoopTime = 0.02;
  
  public static final double kCancoderResolution = 4096;

  // CAN bus names
  public static final String kCanivoreCAN = "CANivore";
  public static final String kRioCAN = "rio";

  /** The key used to access the RobotId name in the RoboRIO's persistent memory. */
  public static final String kRobotIdKey = "RobotId";
}
