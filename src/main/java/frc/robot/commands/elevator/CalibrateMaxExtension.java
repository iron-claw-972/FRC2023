package frc.robot.commands.elevator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;

public class CalibrateMaxExtension extends CommandBase {
  
  private final Elevator m_elevator; 
  
  /**
   * Calibrate the elevator by resetting the encoder after it hits the bottom limit switch.
   */
  public CalibrateMaxExtension(Elevator elevator) {
    m_elevator = elevator; 
    addRequirements(m_elevator);
  }

  @Override
  public void initialize() {
    m_elevator.setPIDEnabled(false); 
  }

  @Override
  public void execute(){
    m_elevator.setMotorPower(ElevatorConstants.kCalibrationPower); 
  }
  
  @Override
  public void end(boolean interrupted) {
    m_elevator.setMotorPower(0); 
    SmartDashboard.putNumber("Max Elevator Extension", m_elevator.getElevatorExtension());
    m_elevator.setPIDEnabled(true);
  }
  
  @Override
  public boolean isFinished() {
    return m_elevator.isTopSwitchTripped();
  }

}


