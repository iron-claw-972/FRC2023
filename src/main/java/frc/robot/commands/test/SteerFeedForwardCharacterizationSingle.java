package frc.robot.commands.test;

import edu.wpi.first.wpilibj.Timer;
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
  
  private final Timer m_timer = new Timer();
  private final Drivetrain m_drive;

  public SteerFeedForwardCharacterizationSingle(Drivetrain drive) {
    m_drive = drive;
    addRequirements(drive);
  }
  
  @Override
  public void initialize() {
    m_timer.start();
    m_feedForwardCharacterizationData = new FeedForwardCharacterizationData();
    m_module = m_drive.getModuleChooser();
  }
  
  @Override
  public void execute() {
    //set voltages
    m_module.setDriveVoltage(0);
    m_module.setSteerVoltage(m_voltage);
    
    // collect data after acceleration time buffer
    if (m_timer.get() > TestConstants.kSteerFeedForwardAccelerationTimeBuffer) {
      m_feedForwardCharacterizationData.add(m_module.getSteerVelocity(), m_voltage);
    }
    // if time past collection time move to increases voltage
    if (m_timer.get() > 
      TestConstants.kSteerFeedForwardAccelerationTimeBuffer + 
      TestConstants.kSteerFeedForwardRecordingTime) {
      m_voltage += TestConstants.kSteerFeedForwardVoltageStep;
      m_timer.reset();
      m_timer.start();
      System.out.println(m_voltage);
    }
  }
  
  @Override
  public void end(boolean interrupted) {
    System.out.println("FINISHED");
    
    m_feedForwardCharacterizationData.process();
    
    //save and print Feed Forward values
    m_drive.getSteerStaticFeedforwardArray()[m_module.getModuleIndex()] = 
      m_feedForwardCharacterizationData.getStatic();
    m_drive.getSteerVelocityFeedforwardArray()[m_module.getModuleIndex()] = 
      m_feedForwardCharacterizationData.getVelocity();
    System.out.println("Static : " + m_feedForwardCharacterizationData.getStatic());
    System.out.println("Velocity : " + m_feedForwardCharacterizationData.getVelocity());
    System.out.println("Variance : " + m_feedForwardCharacterizationData.getVariance());

    m_module.stop();
    
    // update shuffleboard values.
    m_drive.setSteerStaticFeedforwardEntry(
      m_drive.getSteerStaticFeedforwardArray()[
        m_drive.getModuleChooser().getModuleIndex()
      ]
    );
    m_drive.setSteerVelocityFeedforwardEntry(
      m_drive.getSteerVelocityFeedforwardArray()[
          m_drive.getModuleChooser().getModuleIndex()
        ]
      );
  }
  
  @Override
  public boolean isFinished() {
    return m_voltage > TestConstants.kSteerFeedForwardMaxVoltage;
  }
}
