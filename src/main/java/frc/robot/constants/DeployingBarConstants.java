package frc.robot.constants;

public class DeployingBarConstants {
  public static final int kMotor = 0;
  public static final double kMaxRotation = 90, kMinExtension = 0;
  public static final double kP = 1, kI = 0, kD = 0;
  public static final double kTolerance = 0.01;
  public static final double kGearRatio = 0;
  public static final double kDistancePerPulse = 360 / kGearRatio / 2048;

}
