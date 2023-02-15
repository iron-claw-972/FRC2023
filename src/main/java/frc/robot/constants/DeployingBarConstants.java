package frc.robot.constants;

public class DeployingBarConstants {
  public static final int kMotor = 0;
  public static final double kMaxExtension = 0.69, kMinExtension = 0;
  public static final double kP = 1, kI = 0, kD = 0;
  public static final double kTolerance = 0.01;
  public static final double kGearRatio = 0;
  public static final double kSpoolRadius = 0;
  public static final double kDistancePerPulse = 2.0 * Math.PI * kSpoolRadius / kGearRatio / 2048;

}
