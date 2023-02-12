package frc.robot.commands.test;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Module;
import frc.robot.util.FeedForwardCharacterizationData;


/** Add your docs here. */
public class SteerFeedForwardCharacterizationSingle extends CommandBase {
  double m_voltage = 0;
  FeedForwardCharacterizationData m_feedForwardCharacterizationData;
  Module m_module;
  
  Timer m_timer = new Timer();
  Drivetrain m_drive;
  SendableChooser<Module> m_moduleChooser;

  public SteerFeedForwardCharacterizationSingle(Drivetrain drive) {
    m_drive = drive;
    addRequirements(drive);
  }
  
  public void initialize() {
    m_timer.start();
    m_drive.setAllOptimize(false);
    m_feedForwardCharacterizationData = new FeedForwardCharacterizationData();
    this.m_module = m_drive.getModuleChooser().getSelected();
  }
  
  public void execute() {
    runCharacterizationVolts();
    if (m_timer.get() > 0.5) {
      m_feedForwardCharacterizationData.add(m_module.getSteerVelocity(), m_voltage);
    }
    if (m_timer.get() > 2.5) {
      m_voltage += 0.2;
      m_timer.reset();
      m_timer.start();
      System.out.println(m_voltage);
    }
  }
  
  private void runCharacterizationVolts() {
    m_module.setDriveVoltage(0);
    m_module.setSteerVoltage(m_voltage);
    
  }
  
  public void end(boolean interrupted) {
    System.out.println("FINISHED");
    
    m_feedForwardCharacterizationData.print();
    
    m_drive.getSteerStaticFeedforwardArray()[m_module.getModuleType().getID()] = m_feedForwardCharacterizationData.getStatic();
    m_drive.getSteerVelocityFeedforwardArray()[m_module.getModuleType().getID()] = m_feedForwardCharacterizationData.getVelocity();
    System.out.println("Static : " + m_feedForwardCharacterizationData.getStatic());
    System.out.println("Velocity : " + m_feedForwardCharacterizationData.getVelocity());
    System.out.println("Variance : " + m_feedForwardCharacterizationData.getVariance());

    m_module.setDriveVoltage(0);
    m_module.setSteerVoltage(0);
    
    m_drive.getSteerStaticFeedforwardEntry().setDouble(m_drive.getSteerStaticFeedforwardArray()[m_moduleChooser.getSelected().getModuleType().getID()]);
    m_drive.getSteerVelocityFeedforwardEntry().setDouble(m_drive.getSteerVelocityFeedforwardArray()[m_moduleChooser.getSelected().getModuleType().getID()]);
  }
  
  public boolean isFinished() {
    //System.out.println(value > 11);
    return m_voltage > 6;
  }
}
