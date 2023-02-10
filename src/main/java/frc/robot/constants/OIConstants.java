package frc.robot.constants;

// OI stands for Operator Input
public class OIConstants {
  public static final int kDriverJoy = 0;
  public static final int kOperatorJoy = 1;
  public static final int kTestJoy = 2;
  
  public static final double kDeadband = 0.05;

  public static final double kTranslationalSenseitivity = 0.3;
  public static final double kTranslationalExpo = 2;
  public static final double kTranslationalDeadband = 0.05;
  public static final double kTranslationalSlewrate = 4;
  public static final boolean kFieldRelative = true;
  
  public static final double kRotationSenseitiviy = 0.05;
  public static final double kRotationExpo = 4;
  public static final double kRotationDeadband = 0.01;
  public static final double kRotationSlewrate = 10;

  public static final double kHeadingSensitivity = 4;
  public static final double kHeadingExpo = 2;
  public static final double kHeadingDeadband = 0.05;
  public static final boolean kConsantHeadingMagnatuide = false;

  public static final boolean kInvert = false;
}
