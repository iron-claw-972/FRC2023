package frc.robot.commands.elevator;
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
    m_elevator.setSetpointMeters(m_elevatorSetpoint);
    m_elevator.enableDisablePID(true);
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
