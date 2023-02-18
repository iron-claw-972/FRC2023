// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.test;



import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.FeedForwardCharacterizationData;


/** Add your docs here. */
public class DriveFeedForwardCharacterzation extends CommandBase {
  double value = 0;
  FeedForwardCharacterizationData[] m_feedForwardCharacterizationData;
  

  Timer m_timer = new Timer();
  Drivetrain m_drive;

  public DriveFeedForwardCharacterzation(Drivetrain drive) {
    
    this.m_drive = drive;
    addRequirements(drive);
  }

  public void initialize() {
    m_timer.start();
    m_drive.setAllOptimize(false);
    m_feedForwardCharacterizationData = new FeedForwardCharacterizationData[] {
      new FeedForwardCharacterizationData(),
      new FeedForwardCharacterizationData(),
      new FeedForwardCharacterizationData(),
      new FeedForwardCharacterizationData()
    };
  }

  public void execute() {
    runCharacterizationVolts();
    if (m_timer.get() > 0.5) {
      for (int i=0; i<4; i++){
        m_feedForwardCharacterizationData[i].add(m_drive.getDriveVelocities()[i], value); 
      }
      
    }
    if (m_timer.get() >= 2.0) {
      value += 0.2;
      m_timer.reset();
      m_timer.start();
      System.out.println(value);
    }

  }

  private void runCharacterizationVolts() {
    m_drive.driveVoltsTest(value);
  }

  public void end(boolean interrupted) {
    System.out.println("FINISHED");
    for (int i=0; i<4; i++){
      m_feedForwardCharacterizationData[i].process();
    }
    
    for (int i=0; i<4;i++){
      m_drive.getDriveStaticFeedforwardArray()[i] = m_feedForwardCharacterizationData[i].getStatic();
      m_drive.getDriveVelocityFeedforwardArray()[i] = m_feedForwardCharacterizationData[i].getVelocity();
      System.out.println("Static " + i + ": " + m_feedForwardCharacterizationData[i].getStatic());
      System.out.println("Velocity " + i + ": " + m_feedForwardCharacterizationData[i].getVelocity());
    }
    
    m_drive.stop();
  }

  public boolean isFinished() {
    //System.out.println(value > 11);
    return value > 11;
  }

  
}
