package frc.robot;

import java.sql.Driver;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
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
import frc.robot.Robot.RobotId;
import frc.robot.commands.drive.DefaultDriveCommand;
import frc.robot.commands.test.drive.CircleDrive;
import frc.robot.commands.test.drive.DriveFeedForwardCharacterization;
import frc.robot.commands.test.drive.OdometryTestCommand;
import frc.robot.commands.test.drive.SteerFeedForwardCharacterizationSingle;
import frc.robot.commands.test.drive.TestDriveVelocity;
import frc.robot.commands.test.drive.TestHeadingPID;
import frc.robot.commands.test.drive.TestSteerAngle;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.FourBarArm;
import frc.robot.subsystems.Intake;
import frc.robot.util.PathGroupLoader;
import frc.robot.util.Vision;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // Shuffleboard auto chooser
  private final SendableChooser<Command> m_autoCommand = new SendableChooser<>();

  //shuffleboard tabs
  private final ShuffleboardTab m_mainTab = Shuffleboard.getTab("Main");
  private final ShuffleboardTab m_drivetrainTab = Shuffleboard.getTab("Drive");
  private final ShuffleboardTab m_swerveModulesTab = Shuffleboard.getTab("Swerve Modules");
  private final ShuffleboardTab m_autoTab = Shuffleboard.getTab("Auto");
  private final ShuffleboardTab m_controllerTab = Shuffleboard.getTab("Controller");
  private final ShuffleboardTab m_visionTab = Shuffleboard.getTab("Vision");
  private final ShuffleboardTab m_testTab = Shuffleboard.getTab("Test");

  private final Vision m_vision;

  // The robot's subsystems are defined here...
  private final Drivetrain m_drive;
  private final FourBarArm m_arm;
  private final Intake m_intake;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    if (Robot.kRobotId == RobotId.SwerveTest) {
      Constants.configureTestConstants();
    }

    m_vision = new Vision(m_visionTab, Constants.Vision.kCameras);

    // Create Drivetrain, because every robot will have a drivetrain
    m_drive = new Drivetrain(m_drivetrainTab, m_swerveModulesTab, m_vision);

    // Create controllers and subsystems based on robot type
    switch (Robot.kRobotId) {
      case SwerveCompetition:
        m_arm = new FourBarArm();
        m_intake = new Intake();
        OI.configure(m_drive, m_intake, m_arm);
        break;
      case SwerveTest:
        m_arm = null;
        m_intake = null;
        OI.configureDriverControls(m_drive);
        break;
      default:
        DriverStation.reportWarning("Not registering subsystems and controls due to incorrect robot", false);
        m_arm = null;
        m_intake = null;
    }

    // This is really annoying so it's disabled
    DriverStation.silenceJoystickConnectionWarning(true);

    // load paths before auto starts
    PathGroupLoader.loadPathGroups();

    LiveWindow.disableAllTelemetry(); // LiveWindow is causing periodic loop overruns
    LiveWindow.setEnabled(false);
    
    autoChooserUpdate();
    loadCommandSchedulerShuffleboard();
    m_drive.setupDrivetrainShuffleboard();
    m_drive.setupModulesShuffleboard();
    m_vision.setupVisionShuffleboard();
    OI.setupShuffleboard(m_controllerTab);
    
    addTestCommands();

    m_drive.setDefaultCommand(new DefaultDriveCommand(m_drive));
  }

  /**
   * Resets the yaw of the pigeon, unless it has already been reset. Or use force to reset it no matter what.
   * 
   * @param force if the yaw should be reset even if it already has been reset since robot enable.
   */
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
    m_testTab.add("Circle Drive", new CircleDrive(m_drive));
    m_testTab.add("Drive FeedForward", new DriveFeedForwardCharacterization(m_drive));
    m_testTab.add("Steer Single FeedForward", new SteerFeedForwardCharacterizationSingle(m_drive));
    m_testTab.add("Test Drive Velocity", new TestDriveVelocity(m_drive, testEntry));
    m_testTab.add("Heading PID", new TestHeadingPID(m_drive, testEntry));
    m_testTab.add("Steer angle", new TestSteerAngle(m_drive, testEntry));
    m_testTab.add("Odometry Test", new OdometryTestCommand(m_drive, new Transform2d(new Translation2d(1,1), new Rotation2d(Math.PI))));
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