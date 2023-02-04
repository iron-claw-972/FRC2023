package frc.robot.constants;

// OI stands for Operator Input
public class OIConstants {
  public final int kDriverJoy = 0;
  public final int kOperatorJoy = 1;
  public final double kDeadband = 0.05;

  public final double kTranslationalSenseitivity = 0.3;
  public final double kTranslationalExpo = 2;
  public final double kTranslationalDeadband = 0.05;
  public final double kTranslationalSlewrate = 4;
  public final boolean kFieldRelative = true;
  
  public final double kRotationSenseitiviy = 0.05;
  public final double kRotationExpo = 4;
  public final double kRotationDeadband = 0.01;
  public final double kRotationSlewrate = 10;

  public final double kHeadingSenseitiviy = 4;
  public final double kHeadingExpo = 2;
  public final double kHeadingDeadband = 0.05;
}
