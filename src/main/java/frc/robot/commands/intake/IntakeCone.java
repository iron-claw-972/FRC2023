package frc.robot.commands.intake;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.Intake;

public class IntakeCone extends CommandBase {
  
  private final Intake m_intake; 
  
  /**
   * Runs the intake subsystem to intake a cone.
   */
  public IntakeCone(Intake intake) {
    m_intake = intake; 
    addRequirements(m_intake);
  }

  @Override
  public void initialize() {
    m_intake.intake(IntakeConstants.kIntakeSpeed); 
  }

  @Override
  public void end(boolean interrupted) {
    m_intake.stop();
  }
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
    //FIXME: config distance sensor to tell this command to stop once cone is within certain rane of it
  }  
}
