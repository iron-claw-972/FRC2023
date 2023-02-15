package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;

public class ElevatorCalibrationAbsoluteEncoder extends CommandBase {
  Elevator m_elevator; 

  public ElevatorCalibrationAbsoluteEncoder(Elevator elevator) {
    m_elevator = elevator; 
    addRequirements(m_elevator);
  }

  @Override
  public void initialize() {
    m_elevator.set(ElevatorConstants.kMotorEncoderZeroingPower); 
  }

  @Override
  public void end(boolean interrupted) {
    m_elevator.stopMotor();
    m_elevator.setAbsEncoderZeroPos();;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_elevator.getBottomLimitSwitch(); 
  }

}
