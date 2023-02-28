package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.controls.ManualController;
import frc.robot.subsystems.Elevator;

public class ManualElevatorControl extends CommandBase {
  
  private final Elevator m_elevator; 
  private final ManualController m_manualController;
  private double m_currentElevatorPos;
  
  /**
   * Enables manual control of the elevator.
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
    if (m_manualController.returnClampedLeftJoyValue() !=0){
      m_elevator.setPIDEnabled(false); 
      m_elevator.setMotorPower(m_manualController.returnClampedLeftJoyValue());
    }else{
      m_elevator.setPIDEnabled(true); 
      m_currentElevatorPos = m_elevator.getElevatorExtension(); 
      m_elevator.setTargetExtension(m_currentElevatorPos);
    }
  }
}
