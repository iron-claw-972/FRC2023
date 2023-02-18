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

  /**
   * Operator Input Constants
   * @deprecated Instead of frc.robot.constants.Constants.oi use frc.robot.constants.OIConstants
   */
  // @Deprecated
  // public static final OIConstants oi = new OIConstants();

  /**
   * Drive Constants
   * @deprecated Instead of frc.robot.constants.Constants.drive use frc.robot.constants.DriveConstants
   */
  // @Deprecated
  // public static final DriveConstants drive = new DriveConstants();

  /**
   * Autonomous Constants
   * @deprecated Instead of frc.robot.constants.Constants.auto use frc.robot.constants.AutoConstants
   */
  // @Deprecated
  // public static final AutoConstants auto = new AutoConstants();

  /**
   * Falcon Constants
   * @deprecated Instead of frc.robot.constants.Constants.falcon use frc.robot.constants.FalconConstants
   */
  // @Deprecated
  // public static final FalconConstants falcon = new FalconConstants();
  // @Deprecated
  // public static final VisionConstants vision = new VisionConstants();
  // @Deprecated
  // public static final FieldConstants field = new FieldConstants();
}
