package frc.robot.constants;

/**
 * Container class for arm constants.
 */
public class ArmConstants {

  public static final int kMotorId = 5;
  public static final int kAbsEncoderId = 7; 

  public static final double kP = 8;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  public static final double kTolerance = 1.0/200.0;
  public static final double kMinMotorPower = -0.1;
  public static final double kMaxMotorPower = 0.1;

  // TODO: different values for cone/cube?
  public static final double kStowPos = -0.466;
  public static final double kShelfPos = -0.2;
  public static final double kBottomNodePos = -0.2;
  public static final double kMiddleNodePos = -0.2;
  public static final double kTopNodePos = -0.2;
  public static final double kIntakePos = kBottomNodePos;

  public static final double kMaxArmExtensionPos = -0.001; 
}
