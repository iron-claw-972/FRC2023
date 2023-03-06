package frc.robot.constants;

/**
 * Container class for operator input constants.
 */
public class OIConstants {
  public static final int kDriverJoy = 0;
  public static final int kOperatorJoy = 1;
  public static final int kTestJoy = 2;
  public static final int kManualJoy = 3;
  
  public static final double kDeadband = 0.01;

  //TODO: change sensitivity to 1?
  public static final double kTranslationalSensitivity = 1;
  public static final double kTranslationalExpo = 2;
  public static final double kTranslationalDeadband = 0.05;
  public static final double kTranslationalSlewrate = 20;
  public static final boolean kFieldRelative = true;
  
  public static final double kRotationSensitivity = 1;
  public static final double kRotationExpo = 4;
  public static final double kRotationDeadband = 0.01;
  public static final double kRotationSlewrate = 10;

  public static final double kHeadingSensitivity = 4;
  public static final double kHeadingExpo = 2;
  public static final double kHeadingDeadband = 0.05;
  public static final boolean kConstantHeadingMagnitude = false;

  public static final boolean kInvert = false;
  
}
