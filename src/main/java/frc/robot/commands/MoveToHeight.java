package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class MoveToHeight extends CommandBase {
  Elevator m_elevator; 
  double m_elevatorSetpoint; 

  public MoveToHeight(Elevator elevator, double elevatorHeight) {
    m_elevator = elevator; 
    m_elevatorSetpoint = elevatorHeight; 
    addRequirements(m_elevator);
  }

  @Override
  public void execute() {
    System.out.println(m_elevatorSetpoint-m_elevator.getElevatorHeightMeters());
    m_elevator.setSepointMeters(m_elevatorSetpoint); 

  }
}

//Note: this command will never stop running, the motor needs to constantly output power to maintain the elevator's height
