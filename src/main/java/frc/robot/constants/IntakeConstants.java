package frc.robot.constants;

import com.revrobotics.CANSparkMax.IdleMode;

public final class IntakeConstants {
  // motor ports
  public static final int kIntakeMotorId = 0;

  // intake speeds
  public static final double kIntakeCubePower = 0.2;
  public static final double kOuttakeCubePower = -0.2;
  public static final double kIntakeConePower = -0.2;
  public static final double kOuttakeConePower = 0.2;
  public static final double kStopPower = 0;


  // distance sensor thresholds

  public static final int kMotorCurrentLimit = 100;
}