package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class ElevatorCalibrationLimitSwitch extends CommandBase {
  Elevator m_elevator; 

  public ElevatorCalibrationLimitSwitch(Elevator elevator) {
    m_elevator = elevator; 
    addRequirements(m_elevator);
  }

  @Override
  public void initialize() {
    
  }

  @Override
  public void execute() {

    m_elevator.calibrationBottomSwitchNotTripped();

    m_elevator.calibrationBottomSwitchTripped();

    m_elevator.calibrationSetMaxElevatorHeight();
    

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
