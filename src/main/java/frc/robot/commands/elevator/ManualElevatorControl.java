//TODO: Need to figure out a way to ovverride the constant PID adjustment happening in Elevator.java subsystem to run this command


package frc.robot.commands.elevator;

import frc.robot.controls.Operator;
import frc.robot.controls.TestController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class ManualElevatorControl extends CommandBase {
  Elevator m_elevator; 
  double m_currentElevatorPos;
  TestController m_testController;
  /**
   * This command allows for manual control of the elevator. 
   * TODO: get manual control of all subsystems to be exclusive to a third controller
   * @param elevator
   */
  public ManualElevatorControl(Elevator elevator, TestController testController) {
    m_elevator = elevator; 
    m_testController = testController;
    addRequirements(m_elevator);
  }
  @Override
  public void initialize() {
    m_elevator.setEnabled(false); 

  }
  @Override
  public void execute() {
    if ((m_testController.getClampedThrottleValue() > 0) || (m_testController.getClampedThrottleValue() < 0)){
      m_elevator.setEnabled(false); 
      m_elevator.set(m_testController.getClampedThrottleValue());
    }

    if(m_testController.getClampedThrottleValue() == 0){
      m_elevator.setEnabled(true); 
      m_currentElevatorPos = m_elevator.getHeight(); 
      m_elevator.setSetpoint(m_currentElevatorPos);
    }

    
   
  }

  @Override
  public void end(boolean interrupted) {
    
  }
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; 
  }
}
