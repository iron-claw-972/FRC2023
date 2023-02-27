//TODO: Need to figure out a way to ovverride the constant PID adjustment happening in Elevator.java subsystem to run this command


package frc.robot.commands.elevator;

import frc.robot.controls.ManualController;
import frc.robot.controls.Operator;
import frc.robot.controls.TestController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class ManualElevatorControl extends CommandBase {
  Elevator m_elevator; 
  double m_currentElevatorPos;
  ManualController m_manualController;
  /**
   * This command allows for manual control of the elevator. 
   * @param elevator, the elevator subsystem. 
   */
  public ManualElevatorControl(Elevator elevator, ManualController manualController) {
    m_elevator = elevator; 
    m_manualController = manualController;
    addRequirements(m_elevator);
  }

  @Override
  public void initialize() {
    m_elevator.setPIDEnabled(false); 
  }

  @Override
  public void execute() {
    if ((m_manualController.getClampedThrottleValue() > 0) || (m_manualController.getClampedThrottleValue() < 0)){
      m_elevator.setPIDEnabled(false); 
      m_elevator.setSpeedMotorStopWhenLimSwitchesHit(m_manualController.getClampedThrottleValue());
    }
    if(m_manualController.getClampedThrottleValue() == 0){
      m_elevator.setPIDEnabled(true); 
      m_currentElevatorPos = m_elevator.getElevatorExtension(); 
      m_elevator.setTargetExtension(m_currentElevatorPos);
    }
  }
}
