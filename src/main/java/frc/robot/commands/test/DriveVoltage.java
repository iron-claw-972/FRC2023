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

  public void execute(){
    m_drive.testDriveVolts(m_voltsEntry.getDouble(0));
  }

}
