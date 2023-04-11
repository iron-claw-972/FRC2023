package frc.robot.constants;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import frc.robot.commands.scoring.intake.IntakeGamePiece;
import frc.robot.util.GamePieceType;

public final class IntakeConstants {

  // motor port
  public static final int kIntakeMotorId = 9;

  // intake speeds
  public static final double kIntakeCubePower = 0.8;
  public static final double kOuttakeCubePower = -0.6;
  public static final double kIntakeConePower = -0.8;
  public static final double kOuttakeConePower = 0.6;
  public static final double kHoldConePower = -0.5;
  public static final double kHoldGamePieceTimer = 0.5;
  
  public static final double kOuttakeCubePowerAuto = -0.8;

  public static final NeutralMode kNeutralMode = NeutralMode.Brake;

  // Current limits
  public static final int kContinuousCurrentLimit = 25;
  public static final int kPeakCurrentLimit = 55;
  public static final double kPeakCurrentDuration = 0.1;
  public static final boolean kEnableCurrentLimit = true;

  /**
   * The current that the intake motor will stop at in the {@link GamePieceType} command. This needs to be tested.
   */
  // TODO: Set these values to tested values
  public static final double kCubeIntakeCurrentStopPoint = 20;
  public static final double kConeIntakeCurrentStopPoint = 25;
  
  /**
   * How many seconds the outtake will run for. This needs to be tested.
   */
  // TODO: set this to a tested value
  public static final double kOuttakeTime = 0.6;
  /**
   * How many seconds the intake will run for before starting to detect currents for {@link IntakeGamePiece}.
   * This needs to be tested.
   */
  //TODO: set this to a tested value
  public static final double kIntakeStallTime = 0.2;

  /**
  * Distance sensor value when the game piece is in the center
  */
  public static final double kCenterDist = 0.22;

  /**
   * Max distance from the center a game piece can be, the distance sensor will be limited to this amount.
   */
  public static final double kMaxDistanceSensorRange = 0.4;
}