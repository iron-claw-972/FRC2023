package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.commands.DoNothing;
import frc.robot.commands.TestVision;
import frc.robot.commands.TestVision2;
import frc.robot.controls.Driver;
import frc.robot.controls.Operator;
import frc.robot.controls.TestControls;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.Node;
import frc.robot.util.PathGroupLoader;
import frc.robot.util.Vision;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems are defined here...
  private final Drivetrain m_drive = new Drivetrain();

  // Shuffleboard stuff
  SendableChooser<Command> m_autoCommand = new SendableChooser<>();
  SendableChooser<Teams> m_teamChooser = new SendableChooser<>();

  // Vision stuff 
  public static Node selectedNode = null;

  /// Selection values (grid, row, spot)
  public static int[] selectValues = {0,0,0};

  // Timer for clearing array
  public static double selectTime;

  // How much time it should take (in frames)
  public final static double selectTimeAmount=100;

  // Possible teams
  public static enum Teams {BLUE, RED};
  public static Teams team;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // This is really annoying so it's disabled
    DriverStation.silenceJoystickConnectionWarning(true);

    // load paths before auto starts
    PathGroupLoader.loadPathGroups();

    Driver.configureControls(m_drive);
    Operator.configureControls(m_drive);
    TestControls.configureControls(m_drive);

    Vision.setup();

    LiveWindow.disableAllTelemetry(); // LiveWindow is causing periodic loop overruns
    LiveWindow.setEnabled(false);
    
    addTestCommands();
    autoChooserUpdate();
    loadCommandSchedulerShuffleboard();

    // Sets robot pose to 1 meter in front of april tag 2
    m_drive.resetPose(aprilTags[2].getX()-1, aprilTags[2].getY(), Math.PI);
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autoCommand.getSelected();
  }
  public Teams getTeam() {
    return m_teamChooser.getSelected();
  }

  /**
   * Adds the test commands to shuffleboard so they can be run that way.
   */
  public void addTestCommands() {
    ShuffleboardTab tab = Shuffleboard.getTab("Test");
    tab.add("Do Nothing", new DoNothing());
    tab.add("Test vision (forward)", new TestVision(0.1, m_drive));
    tab.add("Test vision (backward)", new TestVision(-0.1, m_drive));
    tab.add("Test vision (forward then backward)", new TestVision2(0.1, 3, m_drive));
    tab.add("Test vision (backward then forward)", new TestVision2(-0.1, 3, m_drive));
    tab.add("Print robot pose", new InstantCommand(()->m_drive.printPose()));
    tab.add("Print pose from vision", new InstantCommand(()->Vision.printEstimate()));
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