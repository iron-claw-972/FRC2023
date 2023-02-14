package frc.robot.constants;

public class ArmConstants {
  // TODO: arm ids - TBD (fake using 5 -- 1-4 have been taken)
  public static final int motorID = 5;

  // TODO: PID values - TBD (fake values for now)
  public static final double kP = 1.0;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  public static final double kTolerance = 0.02;
  public static final double minMotorPower = -0.05;
  public static final double maxMotorPower = +0.05;

  // TODO: distance values - TBD (fake values for now)
  public static final double initialPosition = 0.0;
  public static final double shelfPosition = 0.4;
  public static final double intakePosition = 0.3;
  public static final double middlePosition = 0.4;
  public static final double topPosition = 0.5;
}
