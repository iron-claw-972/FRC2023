package frc.robot.commands.elevator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class MoveToExtension extends CommandBase {
  Elevator m_elevator; 
  double m_targetExtension; 
  /**
   * 
   * @param elevator The elevator subsystem
   * @param targetExtension The target elevator extension required 
   **/
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
