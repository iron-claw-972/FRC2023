package frc.robot.commands.scoring.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.RollerIntake;
import frc.robot.subsystems.RollerIntake.IntakeMode;
import frc.robot.subsystems.RollerIntake.IntakePiece;

public class OuttakeGamePiece extends CommandBase {

  private final RollerIntake m_intake; 
  private final IntakePiece m_heldPiece;
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
  public OuttakeGamePiece(RollerIntake intake, IntakePiece piece) {
    m_intake = intake; 
    m_heldPiece = piece;
    m_timer = new Timer();
    addRequirements(m_intake);
  }

  @Override
  public void initialize() {
    if (m_heldPiece == IntakePiece.CUBE) {
      m_intake.setIntakeMode(IntakeMode.OUTTAKE_CUBE);
    } else if (m_heldPiece == IntakePiece.CONE) {
      m_intake.setIntakeMode(IntakeMode.OUTTAKE_CONE);
    } else {
      cancel();
      return;
    }
    m_timer.reset();
  }

  @Override
  public void end(boolean interrupted) {
    m_intake.setHeldGamePiece(IntakePiece.NONE);
    m_intake.setIntakeMode(IntakeMode.DISABLED);
  }
  
  @Override
  public boolean isFinished() {
    return m_timer.advanceIfElapsed(IntakeConstants.kOuttakeTime);
  }

}
