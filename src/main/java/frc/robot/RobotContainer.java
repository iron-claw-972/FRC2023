  package frc.robot;

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
  import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
  import edu.wpi.first.wpilibj2.command.Command;
  import edu.wpi.first.wpilibj2.command.CommandScheduler;
  import edu.wpi.first.wpilibj2.command.InstantCommand;
  import edu.wpi.first.wpilibj2.command.PrintCommand;
  import frc.robot.Robot.RobotId;
  import frc.robot.commands.DefaultDriveCommand;
  import frc.robot.commands.test.CircleDrive;
  import frc.robot.commands.test.DriveFeedForwardCharacterization;
  import frc.robot.commands.test.OdometryTestCommand;
  import frc.robot.commands.test.SteerFeedForwardCharacterizationSingle;
  import frc.robot.commands.test.TestDriveVelocity;
  import frc.robot.commands.test.TestHeadingPID;
  import frc.robot.commands.test.TestSteerAngle;
  import frc.robot.constants.IntakeConstants;
  import frc.robot.controls.BaseDriverConfig;
  import frc.robot.controls.GameControllerDriverConfig;
  import frc.robot.controls.Operator;
  import frc.robot.subsystems.Drivetrain;
  import frc.robot.subsystems.FourBarArm;
  import frc.robot.subsystems.Intake;
  import frc.robot.util.PathGroupLoader;

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
    private final Drivetrain m_drive;
    private final FourBarArm m_arm;
    private final Intake m_intake;

    // Shuffleboard commands for Intake subsystem


    // Controllers are defined here
    private final BaseDriverConfig m_driver;
    private final Operator m_operator;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer(RobotId robotId) {

      // This is really annoying so it's disabled
      DriverStation.silenceJoystickConnectionWarning(true);

      // load paths before auto starts
      PathGroupLoader.loadPathGroups();

      switch (robotId) {

        case ClassBot2:
          // instance the subsystems
          m_drive = null; 
          m_arm = null;
          m_intake = new Intake();

          // Controllers are defined here
          m_driver = null; 
          m_operator = null;

          SmartDashboard.putData("intake", new InstantCommand(() -> m_intake.intake(IntakeConstants.kIntakeSpeed), m_intake));
          SmartDashboard.putData("Outake", new InstantCommand(() -> m_intake.intake(IntakeConstants.kOuttakeSpeed), m_intake));
          SmartDashboard.putData("Stop", new InstantCommand(() -> m_intake.stop(), m_intake));

          SmartDashboard.putData(CommandScheduler.getInstance());

          break;

        default:    
          // instance the subsystems
          m_drive = new Drivetrain(m_drivetrainTab, m_swerveModulesTab);
          m_arm = new FourBarArm();
          m_intake = new Intake();

          // Controllers are defined here
          m_driver = new GameControllerDriverConfig(m_drive, m_controllerTab, false);
          m_operator = new Operator(m_arm, m_intake);

        m_driver.configureControls();
          m_operator.configureControls();

          LiveWindow.disableAllTelemetry(); // LiveWindow is causing periodic loop overruns
          LiveWindow.setEnabled(false);
      
      
          autoChooserUpdate();
          loadCommandSchedulerShuffleboard();
          m_drive.setupDrivetrainShuffleboard();
          m_drive.setupModulesShuffleboard();
          m_driver.setupShuffleboard();
          
          addTestCommands();

          m_drive.setDefaultCommand(new DefaultDriveCommand(m_drive,m_driver));
          break;
      }
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