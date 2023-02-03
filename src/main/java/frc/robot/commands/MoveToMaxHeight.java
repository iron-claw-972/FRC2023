package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.AngledElevator;

public class MoveToMaxHeight extends CommandBase {
  AngledElevator m_elevator; 


  public MoveToMaxHeight(AngledElevator elevator) {
    m_elevator = elevator; 

    addRequirements(m_elevator);
  
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {

  }

  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
