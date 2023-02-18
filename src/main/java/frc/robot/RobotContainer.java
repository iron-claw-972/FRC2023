package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
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
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.test.CircleDrive;
import frc.robot.commands.test.DriveFeedForwardCharacterization;
import frc.robot.commands.test.OdometryTestCommand;
import frc.robot.commands.test.SteerFeedForwardCharacterizationSingle;
import frc.robot.commands.test.TestDriveVelocity;
import frc.robot.commands.test.TestHeadingPID;
import frc.robot.commands.test.TestSteerAngle;
import frc.robot.controls.BaseDriverConfig;
import frc.robot.controls.GameControllerDriverConfig;
import frc.robot.controls.Operator;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.Node;
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
  private final ShuffleboardTab m_testTab = Shuffleboard.getTab("Test");

  // The robot's subsystems are defined here...
  private final Drivetrain m_drive = new Drivetrain(m_drivetrainTab, m_swerveModulesTab);
  private final FourBarArm m_arm = new FourBarArm();
  private final Intake m_intake = new Intake();

  // Controllers are defined here
  private final BaseDriverConfig m_driver = new GameControllerDriverConfig(m_drive, m_controllerTab, false);
  // private final Operator m_operator = new Operator(m_arm, m_intake);

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

    m_driver.configureControls();
    Operator.configureControls(m_drive, m_arm);

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
    
    
    autoChooserUpdate();
    loadCommandSchedulerShuffleboard();

    // Sets robot pose to 1 meter in front of april tag 2
    m_drive.resetPose(new Pose2d(aprilTags[2].getX()-1, aprilTags[2].getY(), new Rotation2d(Math.PI)));

    m_drive.setupDrivetrainShuffleboard();
    m_drive.setupModulesShuffleboard();
    m_driver.setupShuffleboard();
    
    addTestCommands();

    m_drive.setDefaultCommand(new DefaultDriveCommand(m_drive,m_driver));
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
    ShuffleboardTab tab = Shuffleboard.getTab("Test");
    tab.add("Do Nothing", new DoNothing());
    tab.add("Test vision (forward)", new TestVision(0.1, m_drive));
    tab.add("Test vision (backward)", new TestVision(-0.1, m_drive));
    tab.add("Test vision (forward then backward)", new TestVision2(0.1, 3, m_drive));
    tab.add("Test vision (backward then forward)", new TestVision2(-0.1, 3, m_drive));
    // tab.add("Print robot pose", new InstantCommand(()->m_drive.printPose()));
    tab.add("Print pose from vision", new InstantCommand(()->Vision.printEstimate()));
    tab.add("Align to 0 degrees", new Align(0, m_drive));
    tab.add("Align to 90 degrees", new Align(Math.PI/2, m_drive));
    tab.add("Align to -90 degrees", new Align(-Math.PI/2, m_drive));
    tab.add("Align to 180 degrees", new Align(Math.PI, m_drive));

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