package frc.robot.commands.test;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.TestConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.FeedForwardCharacterizationData;

/**
 * Gathers feedforward data for the drivetrain's drive motors.
 */
public class DriveFeedForwardCharacterization extends CommandBase {

  private double m_voltage = 0;
  private FeedForwardCharacterizationData[] m_feedForwardCharacterizationData;
  
  private Timer m_timer = new Timer();
  private Drivetrain m_drive;
  
  /**
   * Creates a new command.
   * @param drive the drivetrain instance
   */
  public DriveFeedForwardCharacterization(Drivetrain drive) {
    m_drive = drive;
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
    m_drive.driveVoltsTest(m_voltage);
    if (m_timer.get() > TestConstants.kDriveFeedForwardAccelerationTimeBuffer) {
      for (int i=0; i<4; i++) {
        m_feedForwardCharacterizationData[i].add(m_drive.getDriveVelocities()[i], m_voltage); 
      }
    }

    if (m_timer.get() >= TestConstants.kDriveFeedForwardAccelerationTimeBuffer + TestConstants.kDriveFeedForwardRecordingTime) {
      m_voltage += TestConstants.kDriveFeedForwardVoltageStep;
      m_timer.reset();
      m_timer.start();
      System.out.println(m_voltage);
    }
  }
  
  public void end(boolean interrupted) {
    System.out.println("FINISHED");
    for (int i = 0; i < 4; i++) {
      m_feedForwardCharacterizationData[i].process();
    }
    
    for (int i = 0; i < 4; i++) {
      m_drive.getDriveStaticFeedforwardArray()[i] = m_feedForwardCharacterizationData[i].getStatic();
      m_drive.getDriveVelocityFeedforwardArray()[i] = m_feedForwardCharacterizationData[i].getVelocity();
      System.out.println("Static " + i + ": " + m_feedForwardCharacterizationData[i].getStatic());
      System.out.println("Velocity " + i + ": " + m_feedForwardCharacterizationData[i].getVelocity());
      System.out.println("Variance " + i + ": " + m_feedForwardCharacterizationData[i].getVariance());
    }
    
    m_drive.getDriveStaticFeedforwardEntry().setDouble(m_drive.getDriveStaticFeedforwardArray()[m_drive.getModuleChooser().getSelected().getModuleType().getID()]);
    m_drive.getDriveVelocityFeedforwardEntry().setDouble(m_drive.getDriveVelocityFeedforwardArray()[m_drive.getModuleChooser().getSelected().getModuleType().getID()]);
    
    m_drive.stop();
  }
  
  public boolean isFinished() {
    m_drive.setAllOptimize(true);
    //System.out.println(value > 11);
    return m_voltage > TestConstants.kDriveFeedForwardMaxVoltage;
  }
}
