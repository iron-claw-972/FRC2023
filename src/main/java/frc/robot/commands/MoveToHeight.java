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
  public void initialize(){
    //set the elevator PID speed to be clamped within the correct range to get it to move faster
    //This should be done already in ResetEncoderAtBottom() but just in case something happens it's here too. 
    m_elevator.setMotorLimit();

    m_elevator.setSepointMeters(m_elevatorSetpoint);

  }

  @Override
  public void execute() {
    System.out.println(m_elevatorSetpoint-m_elevator.getElevatorHeightMeters());
    m_elevator.setSepointMeters(m_elevatorSetpoint); 

  }

  public void end(){
    

  }
}
