package frc.robot.constants;

import edu.wpi.first.math.util.Units;

/**
 * Container class for arm constants.
 */
public class ArmConstants {

  // TODO: arm ids - TBD (fake using 5 -- 1-4 have been taken)
  public static final int kMotorId = 5;
  public static final int kAbsEncoderId = -1; 

  public static final double kStowedAbsEncoderPos = 0; 
  public static final double kFloorIntakeAbsEncoderPos = 0; 
  public static final double kMiddleConeOuttakeAbsEncoderPos = 0; 
  public static final double kTopConeOuttakeAbsEncoderPos = 0; 
  public static final double kMaxArmExtension = 0; 


  // TODO: PID values - TBD (fake values for now)
  public static final double kP = 0.0;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  public static final double kTolerance = -1; // FIXME
  public static final double kMinMotorPower = -0.1;
  public static final double kMaxMotorPower = 0.1;

  // TODO: distance values - TBD (fake values for now)
  public static final double kInitialPosition = 0.0;
  public static final double kShelfPosition = 0.4;
  public static final double kIntakePosition = 0.3;
  public static final double kMiddlePosition = 0.4;
  public static final double kTopPosition = 0.5;

}
