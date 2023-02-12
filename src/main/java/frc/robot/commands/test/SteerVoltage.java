package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class SteerVoltage extends CommandBase {
  
  Drivetrain m_drive;

  public SteerVoltage(Drivetrain drive) {
    m_drive = drive; 
    addRequirements(m_drive);
  }
  
  @Override
  public void execute() {
    m_drive.steerVoltsTest(m_drive.getRequestedVoltsEntry().getDouble(0));
  }
  
  @Override
  public void end(boolean interrupted) {
    m_drive.stop();
  }
  
}
