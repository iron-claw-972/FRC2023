package frc.robot.commands.scoring.intake;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.RollerIntake;
import frc.robot.subsystems.RollerIntake.IntakeMode;
import frc.robot.util.GamePieceType;

public class OuttakeGamePiece extends CommandBase {

  private final RollerIntake m_intake; 
  private final GamePieceType m_heldPiece;
  private final Timer m_timer;

  /**
   * Spins the outtake for a set amount of time.
   * @param intake the intake subsystem
   */
  public OuttakeGamePiece(RollerIntake intake) {
    this(intake, intake.getHeldGamePiece());
  }

  /**
   * Spins the outtake for a set amount of time.
   * @param intake the intake subsystem
   * @param piece the piece to outtake
   */
  public OuttakeGamePiece(RollerIntake intake, GamePieceType piece) {
    m_intake = intake; 
    m_heldPiece = piece;
    m_timer = new Timer();
    addRequirements(m_intake);
  }

  @Override
  public void initialize() {
    if (m_heldPiece == GamePieceType.CUBE) {
      m_intake.setMode(IntakeMode.OUTTAKE_CUBE);
    } else if (m_heldPiece == GamePieceType.CONE) {
      m_intake.setMode(IntakeMode.OUTTAKE_CONE);
    } else {
      DriverStation.reportWarning("OuttakeGamePiece Command missing GamePieceType", false);
      cancel();
      return;
    }
    m_timer.reset();
  }

  @Override
  public void end(boolean interrupted) {
    m_intake.setHeldGamePiece(GamePieceType.NONE);
    m_intake.setMode(IntakeMode.DISABLED);
  }
  
  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(IntakeConstants.kOuttakeTime);
  }

}
