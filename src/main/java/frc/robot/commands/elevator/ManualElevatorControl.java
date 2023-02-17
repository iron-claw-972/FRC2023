//TODO: Need to figure out a way to ovverride the constant PID adjustment happening in Elevator.java subsystem to run this command


package frc.robot.commands.elevator;

import frc.robot.controls.Operator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class ManualElevatorControl extends CommandBase {
  Elevator m_elevator; 
  /**
   * This command allows for manual control of the elevator. 
   * TODO: get manual control of all subsystems to be exclusive to a third controller
   * @param elevator
   */
  public ManualElevatorControl(Elevator elevator) {
    m_elevator = elevator; 
    addRequirements(m_elevator);
  }
  @Override
  public void initialize() {
    m_elevator.enableDisablePID(false); 
  }
  @Override
  public void execute() {
    m_elevator.set(Operator.getRawThrottleValue()); // rename throttlevalue function to somehting else
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
