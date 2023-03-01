package frc.robot.constants;

/**
 * Container class for arm constants.
 */
public class ArmConstants {

  // TODO: arm ids - TBD (fake using 5 -- 1-4 have been taken)
  public static final int kMotorId = 5;
  public static final int kAbsEncoderId = 7; 



  // TODO: PID values - TBD (fake values for now)
  public static final double kP = 0.2;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  public static final double kTolerance = 1.0/200.0;
  public static final double kMinMotorPower = -0.1;
  public static final double kMaxMotorPower = 0.1;

  // TODO: distance values - TBD (fake values for now)
  public static final double kStowedAbsEncoderPos = 0.0;
  public static final double kShelfPositionAbsEncoderPos = 0;
  public static final double kBottomNodePositionAbsEncoderPos = 0;
  public static final double kMiddleConeOuttakeAbsEncoderPos = 0;
  public static final double kTopConeOuttakeAbsEncoderPos = 0;
  public static final double kMaxArmExtensionAbsEncoderPos = 0; 


}
