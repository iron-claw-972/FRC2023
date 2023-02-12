// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.test;



import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Module;
import frc.robot.util.FeedForwardCharacterizationData;


/** Add your docs here. */
public class SteerFeedForwardCharacterzationSingle extends CommandBase {
  double value = 0;
  FeedForwardCharacterizationData m_feedForwardCharacterizationData;
  Module m_module;

  Timer m_timer = new Timer();
  Drivetrain m_drive;
  SendableChooser<Module> m_moduleChooser;

  public SteerFeedForwardCharacterzationSingle(Drivetrain drive, SendableChooser<Module> moduleChooser) {
    m_drive = drive;
    m_moduleChooser = moduleChooser;
    addRequirements(drive);
  }

  public void initialize() {
    m_timer.start();
    m_drive.setAllOptimize(false);
    m_feedForwardCharacterizationData = new FeedForwardCharacterizationData();
    this.m_module = m_moduleChooser.getSelected();
  }

  public void execute() {
    runCharacterizationVolts();
    if (m_timer.get() > 0.5) {
      m_feedForwardCharacterizationData.add(m_module.getSteerVelocity(), value);
    }
    if (m_timer.get() > 2.5) {
      value += 0.2;
      m_timer.reset();
      m_timer.start();
      System.out.println(value);
    }
  }

  private void runCharacterizationVolts() {

    m_module.setDriveVoltage(0);
    m_module.setSteerVoltage(value);
    
  }

  public void end(boolean interrupted) {
    System.out.println("FINISHED");
 
      m_feedForwardCharacterizationData.print();
    
      m_drive.getSteerStaticFeedforwardArray()[m_module.getModuleType().getID()] = m_feedForwardCharacterizationData.getStatic();
      m_drive.getSteerVelocityFeedforwardArray()[m_module.getModuleType().getID()] = m_feedForwardCharacterizationData.getVelocity();
      System.out.println("Static " + ": " + m_feedForwardCharacterizationData.getStatic());
      System.out.println("Velocity " + ": " + m_feedForwardCharacterizationData.getVelocity());

      m_module.setDriveVoltage(0);
      m_module.setSteerVoltage(0);
  }

  public boolean isFinished() {
    //System.out.println(value > 11);
    return value>6;
  }
}