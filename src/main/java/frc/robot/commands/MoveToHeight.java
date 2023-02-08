package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class MoveToHeight extends CommandBase {
  Elevator m_elevator; 
  double m_elevatorHeight; 
  public MoveToHeight(Elevator elevator, double elevatorHeight) {
    m_elevator = elevator; 
    m_elevatorHeight = elevatorHeight; 
    addRequirements(m_elevator);
  }

  @Override
  public void initialize() {
      
  }

  @Override
  public void execute() {
    
    System.out.println(m_elevator.returnHeightError(m_elevatorHeight)); 

    m_elevator.setElevatorMotorSpeed(m_elevator.returnClampedElevatorPID(m_elevatorHeight)); 
    m_elevator.stopMotorsIfLimitSwitchesTripped();

  }

  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
        //TODO: configure isFinished to get command to end

    return false;
  }

}
