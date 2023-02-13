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
import frc.robot.commands.DoNothing;
import frc.robot.commands.test.TestDriveVelocity;
import frc.robot.constants.OIConstants;
import frc.robot.controls.BaseControllerConfig;
import frc.robot.controls.Driver;
import frc.robot.controls.GameControllerConfig;
import frc.robot.controls.Operator;
import frc.robot.controls.TestControls;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.PathGroupLoader;
import lib.controllers.GameController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems are defined here...
  private final Drivetrain m_drive = new Drivetrain();

  private final BaseControllerConfig m_config = new GameControllerConfig();

  private final GameController controller = new GameController(OIConstants.kDriverJoy);
  

  // Shuffleboard auto chooser
  SendableChooser<Command> m_autoCommand = new SendableChooser<>();

  //shuffleboard tabs
  private ShuffleboardTab m_mainTab = Shuffleboard.getTab("Main");
  private ShuffleboardTab m_drivetrainTab = Shuffleboard.getTab("Drive");
  private ShuffleboardTab m_swerveModulesTab = Shuffleboard.getTab("Swerve Modules");
  private ShuffleboardTab m_autoTab = Shuffleboard.getTab("Auto");
  private ShuffleboardTab m_controllerTab = Shuffleboard.getTab("Controller");
  private ShuffleboardTab m_testTab = Shuffleboard.getTab("Test");

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // This is really annoying so it's disabled
    DriverStation.silenceJoystickConnectionWarning(true);

    // load paths before auto starts
    PathGroupLoader.loadPathGroups();

    
    TestControls.configureControls(m_drive);

    LiveWindow.disableAllTelemetry(); // LiveWindow is causing periodic loop overruns
    LiveWindow.setEnabled(false);

    m_config.configureControls();
    

    addTestCommands();
    autoChooserUpdate();
    loadCommandSchedulerShuffleboard();

    m_drive.setDefaultCommand(new DefaultDriveCommand(m_drive));
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
    
    m_testTab.add("Do Nothing", new DoNothing());

    GenericEntry driveVel = m_swerveModulesTab.add("Set Drive Velocity", 0).getEntry();
    GenericEntry steerVel = m_swerveModulesTab.add("Set Steer Velocity", 0).getEntry();
    m_testTab.add("Test Drive Velocity", new TestDriveVelocity(m_drive, driveVel, steerVel));
  }

  /**
   * Updates the auto chooser on shuffleboard to display what auto routines can be selected for running.
   * 
   * Do Nothing should stay the default, other autos are added with m_autoCommand.addOption()
   */
  public void autoChooserUpdate() {
    m_autoCommand.setDefaultOption("Do Nothing", new PrintCommand("This will do nothing!"));
    // add commands below with: m_autoCommand.addOption("Example", new ExampleCommand());

    
    Shuffleboard.getTab("Auto").add("Auto Chooser", m_autoCommand);
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