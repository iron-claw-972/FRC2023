package frc.robot.commands.elevator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;

public class ResetEncoderAtBottom extends CommandBase {
  Elevator m_elevator; 

  public ResetEncoderAtBottom(Elevator elevator) {
    m_elevator = elevator; 
    addRequirements(m_elevator);
  }

  @Override
  public void initialize() {
    //first lower the power levels
    m_elevator.setMotorLimit(ElevatorConstants.kMotorZeroingLimit);
    //then set the setpoint
    m_elevator.setSetpointMeters(ElevatorConstants.kHeightZeroing);
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

