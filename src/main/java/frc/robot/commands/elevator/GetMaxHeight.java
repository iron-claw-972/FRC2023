package frc.robot.commands.elevator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;

public class GetMaxHeight extends CommandBase {
  Elevator m_elevator; 
  /**
   * Calibrate the elevator by resetting the encoder after it hits the bottom
   * limit switch. 
   * @param elevator
   */
  public GetMaxHeight(Elevator elevator) {
    m_elevator = elevator; 
    addRequirements(m_elevator);
  }

  @Override
  public void initialize() {
    m_elevator.setEnabled(false); 
  }

  @Override
  public void execute(){
    m_elevator.setMotorPower(ElevatorConstants.kCalibrationPower); 
  }
  
  @Override
  public void end(boolean interrupted) {
    m_elevator.setMaxHeight();
    m_elevator.setMotorPower(0); 
    SmartDashboard.putNumber("Max Elevator Height", m_elevator.setMaxHeight());


  }
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_elevator.isTop();
  }
}


