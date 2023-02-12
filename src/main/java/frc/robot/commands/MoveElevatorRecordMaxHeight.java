package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;

public class MoveElevatorRecordMaxHeight extends CommandBase {
  Elevator m_elevator; 

  public MoveElevatorRecordMaxHeight(Elevator elevator) {
    m_elevator = elevator; 
    addRequirements(m_elevator);
  }

  @Override
  public void initialize() {

    m_elevator.set(ElevatorConstants.kElevatorMotorEncoderZeroingPower);
    
  }

  @Override
  public void end(boolean interrupted) {
    m_elevator.stopMotor();
    ElevatorConstants.kElevatorTopHeightMeters = m_elevator.getElevatorHeightMeters();
    
  }

  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_elevator.returnTopLimSwitchCondition() == true){
      return true; 
    }
    return false;
  }

}
