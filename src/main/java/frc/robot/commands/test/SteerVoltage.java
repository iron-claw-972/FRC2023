package frc.robot.commands.test;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class SteerVoltage extends CommandBase {
  
  Drivetrain m_drive;
  GenericEntry m_voltsEntry;
  
  public SteerVoltage(Drivetrain drive, GenericEntry voltsEntry) {
    m_drive = drive; 
    m_voltsEntry = voltsEntry;
    addRequirements(m_drive);
  }
  
  @Override
  public void execute() {
    m_drive.steerVoltsTest(m_voltsEntry.getDouble(0));
  }
  
  @Override
  public void end(boolean interrupted) {
    m_drive.stop();
  }
  
}
