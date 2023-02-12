package frc.robot;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.test.*;
import frc.robot.controls.BaseDriverConfig;
import frc.robot.controls.GameControllerDriverConfig;
import frc.robot.controls.Operator;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.PathGroupLoader;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // Shuffleboard auto chooser
  SendableChooser<Command> m_autoCommand = new SendableChooser<>();

  //shuffleboard tabs
  private ShuffleboardTab m_mainTab = Shuffleboard.getTab("Main");
  private ShuffleboardTab m_drivetrainTab = Shuffleboard.getTab("Drive");
  private ShuffleboardTab m_swerveModulesTab = Shuffleboard.getTab("Swerve Modules");
  private ShuffleboardTab m_autoTab = Shuffleboard.getTab("Auto");
  private ShuffleboardTab m_controllerTab = Shuffleboard.getTab("Controller");
  private ShuffleboardTab m_testTab = Shuffleboard.getTab("Test");

  // The robot's subsystems are defined here...
  private final Drivetrain m_drive = new Drivetrain(m_drivetrainTab, m_swerveModulesTab);


  // Controllers are defined here
  private final BaseDriverConfig m_driver = new GameControllerDriverConfig(m_drive, m_controllerTab, false);
  private final Operator m_operator = new Operator();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // This is really annoying so it's disabled
    DriverStation.silenceJoystickConnectionWarning(true);

    // load paths before auto starts
    PathGroupLoader.loadPathGroups();

    m_driver.configureControls();
    m_operator.configureControls();

    LiveWindow.disableAllTelemetry(); // LiveWindow is causing periodic loop overruns
    LiveWindow.setEnabled(false);
    
    m_drive.setupModulesShuffleboard();
    addTestCommands();
    autoChooserUpdate();
    loadCommandSchedulerShuffleboard();
    m_drive.setupDrivetrainShuffleboard();
    m_driver.setupShuffleboard();

    m_drive.setDefaultCommand(new DefaultDriveCommand(m_drive,m_driver));
  }

  public void initDriveYaw(boolean force) {
    m_drive.initializePigeonYaw(force);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autoCommand.getSelected();
  }

  /**
   * Adds the test commands to shuffleboard so they can be run that way.
   */
  public void addTestCommands() {

    GenericEntry testEntry = m_testTab.add("Test Results", false).getEntry();
    m_testTab.add("Circle Drive", new CircleDrive(m_drive, m_drive.getRequestedDriveVelocityEntry(), m_drive.getRequestedSteerVelocityEntry()));
    m_testTab.add("Drive FeedForawrd", new DriveFeedForwardCharacterzation(m_drive));
    m_testTab.add("Steer All FeedForawrd", new SteerFeedForwardCharacterzationAll(m_drive));
    m_testTab.add("Steer Single FeedForawrd", new SteerFeedForwardCharacterzationSingle(m_drive, m_drive.getModuleChooser()));
    m_testTab.add("Drive Voltage", new DriveVoltage(m_drive, m_drive.getRequestedVoltsEntry()));
    m_testTab.add("Drive Steer", new SteerVoltage(m_drive, m_drive.getRequestedVoltsEntry()));
    m_testTab.add("Test Drive Velocity", new TestDriveVelocity(m_drive, m_drive.getRequestedDriveVelocityEntry(), testEntry));
    m_testTab.add("Heading PID", new TestHeadingPID(m_drive, m_drive.getRequestedSteerAngleEntry()));
    m_testTab.add("Steer angle", new TestSteerAngle(m_drive, m_drive.getRequestedHeadingEntry(), testEntry));
  }

  /**
   * Updates the auto chooser on shuffleboard to display what auto routines can be selected for running.
   * 
   * Do Nothing should stay the default, other autos are added with m_autoCommand.addOption()
   */
  public void autoChooserUpdate() {
    m_autoCommand.setDefaultOption("Do Nothing", new PrintCommand("This will do nothing!"));
    // add commands below with: m_autoCommand.addOption("Example", new ExampleCommand());
    
    m_autoTab.add("Auto Chooser", m_autoCommand);
  }

  /**
   * Loads the command scheduler shuffleboard which will add event markers whenever a command finishes, ends, or is interrupted.
   */
  public void loadCommandSchedulerShuffleboard() {
    // Set the scheduler to log Shuffleboard events for command initialize, interrupt, finish
    CommandScheduler.getInstance().onCommandInitialize(command -> Shuffleboard.addEventMarker("Command initialized", command.getName(), EventImportance.kNormal));
    CommandScheduler.getInstance().onCommandInterrupt(command -> Shuffleboard.addEventMarker("Command interrupted", command.getName(), EventImportance.kNormal));
    CommandScheduler.getInstance().onCommandFinish(command -> Shuffleboard.addEventMarker("Command finished", command.getName(), EventImportance.kNormal));
  }
}