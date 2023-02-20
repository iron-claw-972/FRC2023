package frc.robot.constants;

public class DeployingBarConstants {
  public static final int kMotor = 0;
  public static final int kTalonEncoder = 0;
  public static final int kAbsoluteEncoder = 0;
  public static final int kTopLimitSwitch = 0;
  public static final int kBottomLimitSwitch = 0;
  public static final double kCalibrateSpeed = 0.1;
  public static double kStowPos = 0, kDeployPos = 0;
  public static final double kP = 1, kI = 0, kD = 0;
  public static final double kTolerance = 0.01;
  public static final double kGearRatio = 0;
  public static final double kTalonDistancePerPulse = 360 / kGearRatio / 2048; //Talon Encoder
}
