package frc.robot.constants;

import com.ctre.phoenix.motorcontrol.NeutralMode;
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

  public static final NeutralMode kNeutralMode = NeutralMode.Brake;

  /**
   * The current that the intake motor will stop at in the {@link IntakeGamePiece} command. This needs to be tested.
   */
  public static final double kIntakeCurrentStopPoint = 5;
  
  /**
   * How many seconds the outtake will run for. This needs to be tested.
   */
  public static final double kOuttakeTime = 0.5;

  public static final int kMotorCurrentLimit = 100;
}