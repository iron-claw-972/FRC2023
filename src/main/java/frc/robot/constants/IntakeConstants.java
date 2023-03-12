package frc.robot.constants;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import frc.robot.subsystems.RollerIntake.IntakePiece;

public final class IntakeConstants {
  // motor ports
  public static final int kIntakeMotorId = 9;

  // intake speeds
  // TODO: tune these values
  public static final double kIntakeCubePower = 0.05;
  public static final double kOuttakeCubePower = -0.05;
  public static final double kIntakeConePower = -0.05;
  public static final double kOuttakeConePower = 0.05;

  public static final NeutralMode kNeutralMode = NeutralMode.Brake;

  /**
   * The current that the intake motor will stop at in the {@link IntakePiece} command. This needs to be tested.
   */
  // TODO: Set these values to tested values
  public static final double kCubeIntakeCurrentStopPoint = 5;
  public static final double kConeIntakeCurrentStopPoint = 5;
  
  /**
   * How many seconds the outtake will run for. This needs to be tested.
   */
  // TODO: set this to a tested value
  public static final double kOuttakeTime = 0.5;
}