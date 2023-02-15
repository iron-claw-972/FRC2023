package frc.robot.commands;

import frc.robot.controls.Operator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class ManualElevatorControl extends CommandBase {
  Elevator m_elevator; 

  public ManualElevatorControl(Elevator elevator) {
    m_elevator = elevator; 
    addRequirements(m_elevator);
  }

  @Override
  public void execute() {
    m_elevator.set(Operator.getRawThrottleValue()); // rename throttlevalue function to somehting else
    m_elevator.stopMotorsIfLimitSwitchesTripped(Operator.getRawThrottleValue());
  }

  @Override
  public void end(boolean interrupted) {
    m_elevator.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
