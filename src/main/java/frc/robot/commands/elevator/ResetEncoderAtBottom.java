package frc.robot.commands.elevator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;

public class ResetEncoderAtBottom extends CommandBase {
  Elevator m_elevator; 
  /**
   * First part of calibration sequence -- reset the elevator by resetting the encoder after it hits the bottom
   * limit switch. 
   * @param elevator
   */
  public ResetEncoderAtBottom(Elevator elevator) {
    m_elevator = elevator; 
    addRequirements(m_elevator);
  }

  @Override
  public void initialize() {
    m_elevator.setPIDEnabled(false); 
  }

  @Override
  public void execute(){
    m_elevator.setMotorPower(-ElevatorConstants.kCalibrationPower); 
  }
  
  @Override
  public void end(boolean interrupted) {
    m_elevator.resetTalonEncoder();
    m_elevator.setMotorPower(0); 

  }
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_elevator.isBottomSwitchTripped();
  }
}


