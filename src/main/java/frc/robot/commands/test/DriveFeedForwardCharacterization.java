package frc.robot.commands.test;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
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
  
  private final Timer m_timer = new Timer();
  private final Drivetrain m_drive;
  
  public DriveFeedForwardCharacterization(Drivetrain drive) {
    m_drive = drive;
    addRequirements(drive);
  }
  
  @Override
  public void initialize() {
    m_timer.start();
    m_feedForwardCharacterizationData = new FeedForwardCharacterizationData[] {
      new FeedForwardCharacterizationData(),
      new FeedForwardCharacterizationData(),
      new FeedForwardCharacterizationData(),
      new FeedForwardCharacterizationData()
    };
  }
  
  @Override
  public void execute() {
    for (int i = 0; i < 4; i++) {
      m_drive.m_modules[i].setDriveVoltage(m_voltage);
    }
    m_drive.m_modules[0].setAngle(new Rotation2d(Units.degreesToRadians(135)));
    m_drive.m_modules[1].setAngle(new Rotation2d(Units.degreesToRadians(45)));
    m_drive.m_modules[2].setAngle(new Rotation2d(Units.degreesToRadians(225)));
    m_drive.m_modules[3].setAngle(new Rotation2d(Units.degreesToRadians(315)));

    if (m_timer.get() > TestConstants.kDriveFeedForwardAccelerationTimeBuffer) {
      for (int i=0; i<4; i++) {
        m_feedForwardCharacterizationData[i].add(
          m_drive.m_modules[i].getState().speedMetersPerSecond, m_voltage
        );
      }
    }

    if (m_timer.get() >= 
      TestConstants.kDriveFeedForwardAccelerationTimeBuffer + 
      TestConstants.kDriveFeedForwardRecordingTime) {
      m_voltage += TestConstants.kDriveFeedForwardVoltageStep;
      m_timer.reset();
      m_timer.start();
      System.out.println(m_voltage);
    }
  }
  
  @Override
  public void end(boolean interrupted) {
    System.out.println("FINISHED");
    for (int i = 0; i < 4; i++) {
      m_feedForwardCharacterizationData[i].process();
    }
    
    for (int i = 0; i < 4; i++) {
      m_drive.getDriveStaticFeedforwardArray()[i] = 
        m_feedForwardCharacterizationData[i].getStatic();
      m_drive.getDriveVelocityFeedforwardArray()[i] = 
        m_feedForwardCharacterizationData[i].getVelocity();
      System.out.println("Static " + i + ": " + 
        m_feedForwardCharacterizationData[i].getStatic());
      System.out.println("Velocity " + i + ": " + 
        m_feedForwardCharacterizationData[i].getVelocity());
      System.out.println("Variance " + i + ": " + 
        m_feedForwardCharacterizationData[i].getVariance());
    }
    
    m_drive.setDriveStaticFeedforwardEntry(
      m_drive.getDriveStaticFeedforwardArray()[m_drive.getModuleChooser().getModuleIndex()]
    );
    m_drive.setDriveVelocityFeedforwardEntry(
      m_drive.getDriveVelocityFeedforwardArray()[m_drive.getModuleChooser().getModuleIndex()]
    );
    
    m_drive.stop();
  }
  
  @Override
  public boolean isFinished() {
    return m_voltage > TestConstants.kDriveFeedForwardMaxVoltage;
  }
}
