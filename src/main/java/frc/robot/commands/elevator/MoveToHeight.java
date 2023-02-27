package frc.robot.commands.elevator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class MoveToHeight extends CommandBase {
  Elevator m_elevator; 
  double m_targetHeight; 
  /**
   * This command allows the elvator to move to the heights 
   * of the bottom, middle, and top nodes, or any height that we choose. 
   * 
   * @param elevator
   * @param targetHeight
   */
  public MoveToHeight(Elevator elevator, double targetHeight) {
    m_elevator = elevator; 
    m_targetHeight = targetHeight; 
    addRequirements(m_elevator);
  }

  @Override
  public void initialize(){
    m_elevator.setTargetHeight(m_targetHeight); 
    m_elevator.setPIDEnabled(true);
    m_elevator.resetPID();
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("MoveTOHeight Command has FINISHEDDDDDDDDDDDDDDDDDDDDDDDDDDDDDD"); 
    
   
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_elevator.atSetpoint(); 
    
   
  }
}
