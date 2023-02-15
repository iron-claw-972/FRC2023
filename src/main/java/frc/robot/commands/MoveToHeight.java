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
  public void execute() {
    System.out.println(m_elevator.getHeightError(m_elevatorHeight)); 
    m_elevator.set(m_elevator.getClampedElevatorPID(m_elevatorHeight)); 
    m_elevator.stopMotorsIfLimitSwitchesTripped(m_elevator.getClampedElevatorPID(m_elevatorHeight));
  }
}

//Note: this command will never stop running, the motor needs to constantly output power to maintain the elevator's height
