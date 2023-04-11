package frc.robot.commands.scoring.intake;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeMode;


public class HoldCone extends CommandBase {

  private Intake m_intake;
  private final Timer m_timer; 

  public HoldCone(Intake intake) {
    m_intake = intake;
    m_timer = new Timer();
  }

  @Override
  public void initialize() {
    m_intake.setMode(IntakeMode.HOLD_GAME_PIECE);
    m_timer.reset();
    m_timer.start();
  }

  @Override
  public boolean isFinished() {
    return m_timer.get() > IntakeConstants.kHoldGamePieceTimer; 
  }

  @Override
  public void end(boolean interrupted) {
    m_intake.setMode(IntakeMode.DISABLED);
  }

}