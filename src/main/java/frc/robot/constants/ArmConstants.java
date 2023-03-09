package frc.robot.constants;

/**
 * Container class for arm constants.
 */
public class ArmConstants {

  public static final int kMotorId = 5;
  public static final int kAbsEncoderId = 7; 

  public static final double kP = 2;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  public static final double kTolerance = 0.1;
  public static final double kMinMotorPower = -0.3;
  public static final double kMaxMotorPower = 0.3;

  // TODO: different values for cone/cube?
  public static final double kStowPos = 0.96;
  public static final double kBottomNodePos = 0.49;
  public static final double kMiddleNodePos = 0.8;
  public static final double kTopNodePos = 0.712;
  public static final double kIntakePos = kBottomNodePos;
  public static final double kDunkPos = 0.55;
  public static final double kShelfPos = kBottomNodePos;

  public static final double kMaxArmExtensionPos = kIntakePos; 
}
