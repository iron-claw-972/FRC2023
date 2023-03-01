package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;

public class CalibrateElevator extends CommandBase {
  
  private final Elevator m_elevator; 
  
  /**
   * First part of calibration sequence. Resets the elevator by resetting the encoder after it hits the bottom limit switch. 
   */
  public CalibrateElevator(Elevator elevator) {
    m_elevator = elevator; 
    addRequirements(m_elevator);
  }

  @Override
  public void initialize() {
    m_elevator.setPIDEnabled(false); 
    m_elevator.setMotorPower(-ElevatorConstants.kCalibrationPower); 
  }

  @Override
  public void end(boolean interrupted) {
    m_elevator.resetTalonEncoder();
    m_elevator.setMotorPower(0); 
  }
  
  @Override
  public boolean isFinished() {
    return m_elevator.isBottomSwitchTripped();
  }
}