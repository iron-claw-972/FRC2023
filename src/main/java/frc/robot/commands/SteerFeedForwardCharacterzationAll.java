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
public class SteerFeedForwardCharacterzationAll extends CommandBase {
  double value = 0;
  FeedForwardCharacterizationData[] m_feedForwardCharacterizationData;
  

  Timer m_timer = new Timer();
  Drivetrain m_drive;
  int m_module;

  public SteerFeedForwardCharacterzationAll(Drivetrain drive) {
    
    this.m_drive = drive;
    addRequirements(drive);
  }

  public void initialize() {
    m_timer.reset();
    m_timer.start();
    m_drive.setAllOptimize(false);
    m_feedForwardCharacterizationData = new FeedForwardCharacterizationData[] {
      new FeedForwardCharacterizationData(),
      new FeedForwardCharacterizationData(),
      new FeedForwardCharacterizationData(),
      new FeedForwardCharacterizationData()
    };
    m_module = 0;
  }

  public void execute() {
    if (m_module > 3) return;

    runCharacterizationVolts();
    if (m_timer.get() > 0.5) {
        m_feedForwardCharacterizationData[m_module].add(Robot.drive.m_modules[m_module].getSteerVelocity(), value);
      
    }
    if (m_timer.get() > 2.5) {
      value += 0.2;
      m_timer.reset();
      m_timer.start();
      System.out.println(value);
    }

    if (value > 6){
      m_module++;
      value = 0;
    }

  }

  private void runCharacterizationVolts() {
    for (int i = 0; i < 4; i++) {
      Robot.drive.m_modules[i].setDriveVoltage(0);
      if (m_module == i){
        Robot.drive.m_modules[i].setSteerVoltage(value);
      } else {
        Robot.drive.m_modules[i].setSteerVoltage(0);
      }
    }
  }

  public void end(boolean interrupted) {
    System.out.println("FINISHED");
    for (int i=0; i<4; i++){
      m_feedForwardCharacterizationData[i].print();
    }
    
    for (int i=0; i<4;i++){
      Robot.shuffleboard.m_steerStaticFeedForwardSaver.replace(Robot.drive.m_modules[i], m_feedForwardCharacterizationData[i].getStatic());
      Robot.shuffleboard.m_steerVelFeedForwardSaver.replace(Robot.drive.m_modules[i], m_feedForwardCharacterizationData[i].getVelocity());
      System.out.println("Static " + i + ": " + m_feedForwardCharacterizationData[i].getStatic());
      System.out.println("Velocity " + i + ": " + m_feedForwardCharacterizationData[i].getVelocity());
    }
    
    for (int i = 0; i < 4; i++) {
      Robot.drive.m_modules[i].setDriveVoltage(0);
      Robot.drive.m_modules[i].setSteerVoltage(0);
    }
    
  }

  public boolean isFinished() {
    //System.out.println(value > 11);
    return m_module > 3;
  }

  
}
