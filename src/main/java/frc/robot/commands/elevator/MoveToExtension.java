package frc.robot.commands.elevator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class MoveToExtension extends CommandBase {
  
  private final Elevator m_elevator; 
  private final double m_targetExtension; 
  
  /**
   * Moves the elevator to a specific extension point.
   */
  public MoveToExtension(Elevator elevator, double targetExtension) {
    m_elevator = elevator; 
    m_targetExtension = targetExtension; 
    addRequirements(m_elevator);
  }

  @Override
  public void initialize(){
    m_elevator.setTargetExtension(m_targetExtension); 
    m_elevator.setPIDEnabled(true);
    m_elevator.resetPID();
  }

}
