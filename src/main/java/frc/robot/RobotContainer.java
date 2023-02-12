package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.commands.DoNothing;
import frc.robot.controls.Driver;
import frc.robot.controls.Operator;
import frc.robot.controls.TestControls;
import frc.robot.subsystems.DeployingBar;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.PathGroupLoader;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems are defined here...
  private final Drivetrain m_drive = new Drivetrain();
  private final DeployingBar m_deployingbar = new DeployingBar();

  // Shuffleboard stuff
  SendableChooser<Command> m_autoCommand = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // This is really annoying so it's disabled
    DriverStation.silenceJoystickConnectionWarning(true);

    // load paths before auto starts
    PathGroupLoader.loadPathGroups();

    Driver.configureControls(m_drive);
    Operator.configureControls(m_deployingbar);
    TestControls.configureControls(m_drive);

    LiveWindow.disableAllTelemetry(); // LiveWindow is causing periodic loop overruns
    LiveWindow.setEnabled(false);
    
    addTestCommands();
    autoChooserUpdate();
    loadCommandSchedulerShuffleboard();
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
    ShuffleboardTab tab = Shuffleboard.getTab("Test");
    tab.add("Do Nothing", new DoNothing());
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