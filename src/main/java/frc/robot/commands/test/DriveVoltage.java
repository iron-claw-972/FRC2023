package frc.robot.commands.test;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveVoltage extends CommandBase {
  
  Drivetrain m_drive;
  GenericEntry m_voltsEntry;

  public DriveVoltage(Drivetrain drive, GenericEntry voltsEntry){
    m_drive = drive; 
    addRequirements(m_drive);
  }
  
  @Override
  public void execute(){
    m_drive.driveVoltsTest(m_voltsEntry.getDouble(0));
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.stop();
  }

}
