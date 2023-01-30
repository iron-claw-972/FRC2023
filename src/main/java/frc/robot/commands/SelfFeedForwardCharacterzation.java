// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;



import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.FeedForwardCharacterizationData;


/** Add your docs here. */
public class SelfFeedForwardCharacterzation extends CommandBase {
  double value = 0;
  FeedForwardCharacterizationData[] m_feedForwardCharacterizationData =new FeedForwardCharacterizationData[] {
    new FeedForwardCharacterizationData(),
    new FeedForwardCharacterizationData(),
    new FeedForwardCharacterizationData(),
    new FeedForwardCharacterizationData()
  };
  

  Timer m_timer = new Timer();
  Drivetrain m_drive;

  public SelfFeedForwardCharacterzation(Drivetrain drive) {
    
    this.m_drive = drive;
    addRequirements(drive);
  }

  public void initialize() {
    m_timer.start();
    m_drive.setAllOptimize(false);
  }

  public void execute() {
    runcharacterazationVolts();
    if (m_timer.get() > 0.5) {
      for (int i=0; i<4; i++){
        m_feedForwardCharacterizationData[i].add(Robot.drive.m_modules[i].getDriveVelocity(), value); 
      }
      
    }
    if (m_timer.get() >= 2.0) {
      value += 0.2;
      m_timer.reset();
      m_timer.start();
    System.out.println(value);
    }

  }

  private void runcharacterazationVolts() {
    for (int i = 0; i < 4; i++) {
      Robot.drive.m_modules[i].setDriveVoltage(value);
    }
    Robot.drive.m_modules[0].setSteerAngle(new Rotation2d(Units.degreesToRadians(135)));
    Robot.drive.m_modules[1].setSteerAngle(new Rotation2d(Units.degreesToRadians(45)));
    Robot.drive.m_modules[2].setSteerAngle(new Rotation2d(Units.degreesToRadians(225)));
    Robot.drive.m_modules[3].setSteerAngle(new Rotation2d(Units.degreesToRadians(315)));
  }

  public void end(boolean interrupted) {
    System.out.println("FINISHED");
    for (int i=0; i<4; i++){
      m_feedForwardCharacterizationData[i].print();
    
    }
    
    for (int i=0; i<4;i++){
      Robot.shuffleboard.m_staticModulesSaver.replace(Robot.drive.m_modules[i], m_feedForwardCharacterizationData[i].getSatic());
      Robot.shuffleboard.m_velModulesSaver.replace(Robot.drive.m_modules[i], m_feedForwardCharacterizationData[i].getVelocity());
      System.out.println("Static " + i + ": " + m_feedForwardCharacterizationData[i].getSatic());
      System.out.println("Velocity " + i + ": " + m_feedForwardCharacterizationData[i].getVelocity());
      


    }
    
    for (int i = 0; i < 4; i++) {
      // Robot.drive.m_modules[i].setDriveVoltage(0);
    }
    Robot.drive.m_modules[0].setSteerAngle(new Rotation2d(Units.degreesToRadians(135)));
    Robot.drive.m_modules[1].setSteerAngle(new Rotation2d(Units.degreesToRadians(45)));
    Robot.drive.m_modules[2].setSteerAngle(new Rotation2d(Units.degreesToRadians(225)));
    Robot.drive.m_modules[3].setSteerAngle(new Rotation2d(Units.degreesToRadians(315)));
  }

  public boolean isFinished() {
    //System.out.println(value > 11);
    return value > 11;
  }

  
}
