package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.commands.Align;
import frc.robot.commands.DoNothing;
import frc.robot.commands.TestVision;
import frc.robot.commands.TestVision2;
import frc.robot.controls.Driver;
import frc.robot.controls.Operator;
import frc.robot.controls.TestControls;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.Node;
import frc.robot.subsystems.FourBarArm;
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
  private final FourBarArm m_arm = new FourBarArm();

  // Shuffleboard stuff
  SendableChooser<Command> m_autoCommand = new SendableChooser<>();

  // Where the robot will score.
  public static Node selectedNode = null;

  // Array of april tags. The index of the april tag in the array is equal to its id, and aprilTags[0] is null.
  public final static Pose3d[] aprilTags = new Pose3d[9];

  // 2D arrays of nodes. blueNodes[3][1] will return the top row cone node on the far left side (from the perspective of the driver)
  public final static Node[][] blueNodes = new Node[4][];
  public final static Node[][] redNodes = new Node[4][];



  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // This is really annoying so it's disabled
    DriverStation.silenceJoystickConnectionWarning(true);

    // load paths before auto starts
    PathGroupLoader.loadPathGroups();

    Driver.configureControls(m_drive, m_arm);
    Operator.configureControls(m_drive, m_arm);
    TestControls.configureControls(m_drive);

    Vision.setup(m_drive);

    // Puts April tags in array
    for(int i = 1; i <= 8; i++){
      aprilTags[i] = Vision.getTagPose(i);
    }

    // Puts nodes in arrays
    for(int i = 1; i <= 3; i++){
      blueNodes[i] = new Node[10];
      redNodes[i] = new Node[10];
      for(int j = 1; j <= 9; j++){
        blueNodes[i][j] = new Node(Alliance.Blue, i, j);
        redNodes[i][j] = new Node(Alliance.Red, i, j);
      }
    }

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
    tab.add("Align to 0 degrees", new Align(0, m_drive));
    tab.add("Align to 90 degrees", new Align(Math.PI/2, m_drive));
    tab.add("Align to -90 degrees", new Align(-Math.PI/2, m_drive));
    tab.add("Align to 180 degrees", new Align(Math.PI, m_drive));
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