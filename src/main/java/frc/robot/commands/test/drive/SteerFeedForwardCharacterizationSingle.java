package frc.robot.commands.test.drive;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Module;
import frc.robot.util.FeedForwardCharacterizationData;

/**
 * Gathers feedforward data for the drivetrain's steer motors.
 */
public class SteerFeedForwardCharacterizationSingle extends CommandBase {
  private final double kSteerFeedForwardVoltageStep = 0.2;
  private final double kSteerFeedForwardMaxVoltage = 6;
  private final double kSteerFeedForwardAccelerationTimeBuffer = 0.5;
  private final double kSteerFeedForwardRecordingTime = 2;

  private double m_voltage = 0;
  private FeedForwardCharacterizationData m_feedForwardCharacterizationData;
  private Module m_module;
  
  private final Timer m_timer = new Timer();
  private final Drivetrain m_drive;
  private SendableChooser<Module> m_moduleChooser;

  public SteerFeedForwardCharacterizationSingle(Drivetrain drive) {
    m_drive = drive;
    addRequirements(drive);
  }
  
  @Override
  public void initialize() {
    m_timer.start();
    m_feedForwardCharacterizationData = new FeedForwardCharacterizationData();
    m_module = m_drive.getModuleChooser().getSelected();
  }
  
  @Override
  public void execute() {
    //set voltages
    m_module.setDriveVoltage(0);
    m_module.setSteerVoltage(m_voltage);
    
    // collect data after acceleration time buffer
    if (m_timer.get() > kSteerFeedForwardAccelerationTimeBuffer) {
      m_feedForwardCharacterizationData.add(m_module.getSteerVelocity(), m_voltage);
    }
    // if time past collection time move to increases voltage
    if (m_timer.get() > kSteerFeedForwardAccelerationTimeBuffer + kSteerFeedForwardRecordingTime) {
      m_voltage += kSteerFeedForwardVoltageStep;
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
    m_drive.getSteerStaticFeedforwardArray()[m_module.getModuleType().getID()] = m_feedForwardCharacterizationData.getStatic();
    m_drive.getSteerVelocityFeedforwardArray()[m_module.getModuleType().getID()] = m_feedForwardCharacterizationData.getVelocity();
    System.out.println("Static : " + m_feedForwardCharacterizationData.getStatic());
    System.out.println("Velocity : " + m_feedForwardCharacterizationData.getVelocity());
    System.out.println("Variance : " + m_feedForwardCharacterizationData.getVariance());

    m_module.stop();
    
    // update shuffleboard values.
    m_drive.getSteerStaticFeedforwardEntry().setDouble(
      m_drive.getSteerStaticFeedforwardArray()[
        m_moduleChooser.getSelected().getModuleType().getID()
      ]
    );
    m_drive.getSteerVelocityFeedforwardEntry().setDouble(
      m_drive.getSteerVelocityFeedforwardArray()[
          m_moduleChooser.getSelected().getModuleType().getID()
        ]
      );
  }
  
  @Override
  public boolean isFinished() {
    return m_voltage > kSteerFeedForwardMaxVoltage;
  }
}
