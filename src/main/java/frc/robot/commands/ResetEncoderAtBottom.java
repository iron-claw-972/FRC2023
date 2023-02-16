package frc.robot.commands;
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
    m_elevator.setSepointMeters(ElevatorConstants.kHeightZeroing);
  }
}

