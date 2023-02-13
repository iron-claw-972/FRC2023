package frc.robot.commands.test;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.TestConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Module;
import frc.robot.util.FeedForwardCharacterizationData;

/**
 * Gathers feedforward data for the drivetrain's steer motors.
 */
public class SteerFeedForwardCharacterizationSingle extends CommandBase {

  private double m_voltage = 0;
  private FeedForwardCharacterizationData m_feedForwardCharacterizationData;
  private Module m_module;
  
  private Timer m_timer = new Timer();
  private Drivetrain m_drive;
  private SendableChooser<Module> m_moduleChooser;

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
    m_module.setDriveVoltage(0);
    m_module.setSteerVoltage(m_voltage);
    
    if (m_timer.get() > TestConstants.kSteerFeedForwardAccelerationTimeBuffer) {
      m_feedForwardCharacterizationData.add(m_module.getSteerVelocity(), m_voltage);
    }
    if (m_timer.get() > TestConstants.kSteerFeedForwardAccelerationTimeBuffer + TestConstants.kSteerFeedForwardRecordingTime) {
      m_voltage += TestConstants.kSteerFeedForwardVoltageStep;
      m_timer.reset();
      m_timer.start();
      System.out.println(m_voltage);
    }
  }
  
  public void end(boolean interrupted) {
    System.out.println("FINISHED");
    
    m_feedForwardCharacterizationData.process();
    
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
    m_drive.setAllOptimize(true);
    //System.out.println(value > 11);
    return m_voltage > TestConstants.kSteerFeedForwardMaxVoltage;
  }
}
