package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.GenericEntry;
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
import frc.robot.Robot.RobotId;
import frc.robot.commands.BalanceCommand;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.auto.DepositThenPath;
import frc.robot.commands.auto.PathPlannerCommand;
import frc.robot.commands.scoring.PositionIntake;
import frc.robot.commands.scoring.PositionIntake.Position;
import frc.robot.commands.scoring.intake.IntakeGamePiece;
import frc.robot.commands.scoring.intake.OuttakeGamePiece;
import frc.robot.constants.VisionConstants;
import frc.robot.constants.swerve.DriveConstants;
import frc.robot.controls.BaseDriverConfig;
import frc.robot.controls.GameControllerDriverConfig;
import frc.robot.controls.ManualController;
import frc.robot.controls.Operator;
import frc.robot.controls.TestController;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.FourBarArm;
import frc.robot.subsystems.RollerIntake;
import frc.robot.util.GamePieceType;
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
  private final SendableChooser<GamePieceType> m_preloadedGamePiece = new SendableChooser<>();

  //shuffleboard tabs
  private final ShuffleboardTab m_mainTab = Shuffleboard.getTab("Main");
  private final ShuffleboardTab m_drivetrainTab = Shuffleboard.getTab("Drive");
  private final ShuffleboardTab m_swerveModulesTab = Shuffleboard.getTab("Swerve Modules");
  private final ShuffleboardTab m_autoTab = Shuffleboard.getTab("Auto");
  private final ShuffleboardTab m_controllerTab = Shuffleboard.getTab("Controller");
  private final ShuffleboardTab m_visionTab = Shuffleboard.getTab("Vision");
  private final ShuffleboardTab m_testTab = Shuffleboard.getTab("Test");
  private final ShuffleboardTab m_elevatorTab = Shuffleboard.getTab("Elevator");
  private final ShuffleboardTab m_intakeTab = Shuffleboard.getTab("Intake");

  private final Vision m_vision;

  // The robot's subsystems are defined here...
  private final Drivetrain m_drive;
  private final FourBarArm m_arm;
  private final RollerIntake m_intake;
  private final Elevator m_elevator;

  // Controllers are defined here
  private final BaseDriverConfig m_driver;
  private final Operator m_operator;
  private final TestController m_testController;
  private final ManualController m_manualController;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer(RobotId robotId) {

    // PowerDistribution m_PDModule = new PowerDistribution(1, ModuleType.kRev);
    // m_PDModule.clearStickyFaults();

    switch (robotId) {
      case SwerveCompetition:
        // Update drive constants based off of robot type
        DriveConstants.update(robotId);
        VisionConstants.update(robotId);

        m_vision = new Vision(m_visionTab, VisionConstants.kCameras);

        // Create Drivetrain
        m_drive = new Drivetrain(m_drivetrainTab, m_swerveModulesTab, m_vision);
        m_driver = new GameControllerDriverConfig(m_drive, m_controllerTab, false);
    
        m_arm = new FourBarArm();
        m_intake = new RollerIntake(m_intakeTab);
        m_elevator = new Elevator(m_elevatorTab, () -> m_intake.containsGamePiece());
  
        m_operator = new Operator();
        m_testController = new TestController(m_arm, m_intake, m_elevator);
        m_manualController = new ManualController(m_arm, m_intake, m_elevator);
  
        m_operator.configureControls(m_arm, m_intake, m_elevator, m_vision);
        m_testController.configureControls();
        m_manualController.configureControls();
  
        // load paths before auto starts
        PathGroupLoader.loadPathGroups();

        // add camera display
        CameraServer.startAutomaticCapture();

        m_driver.configureControls();

        m_vision.setupVisionShuffleboard();
        m_driver.setupShuffleboard();

        m_drive.setDefaultCommand(new DefaultDriveCommand(m_drive, m_driver));

        break;

      case SwerveTest:
        // Update drive constants based off of robot type
        DriveConstants.update(robotId);
        VisionConstants.update(robotId);

        m_vision = new Vision(m_visionTab, VisionConstants.kCameras);

        // Create Drivetrain, because every robot will have a drivetrain
        m_drive = new Drivetrain(m_drivetrainTab, m_swerveModulesTab, m_vision);
        m_driver = new GameControllerDriverConfig(m_drive, m_controllerTab, false);

        DriverStation.reportWarning("Not registering subsystems and controls due to incorrect robot", false);

        // TODO: construct dummy subsystems so SwerveTest can run all auto routines
        m_arm = null;
        m_intake = null;
        m_elevator = null;
  
        m_operator = null;
        m_testController = null;
        m_manualController = null;

        // load paths before auto starts
        PathGroupLoader.loadPathGroups();

        // add camera display
        CameraServer.startAutomaticCapture();

        m_driver.configureControls();

        m_vision.setupVisionShuffleboard();
        m_driver.setupShuffleboard();

        m_drive.setDefaultCommand(new DefaultDriveCommand(m_drive, m_driver));
        
        break;

      default:
        DriverStation.reportWarning("Not registering subsystems and controls due to incorrect robot", false);

        m_vision = null;

        m_driver = null;
        m_drive = null;

        m_arm = null;
        m_intake = null;
        m_elevator = null;

        m_operator = null;
        m_testController = null;
        m_manualController = null;
        break;
    }

    // This is really annoying so it's disabled
    DriverStation.silenceJoystickConnectionWarning(true);

    LiveWindow.disableAllTelemetry(); // LiveWindow is causing periodic loop overruns
    LiveWindow.setEnabled(false);
    
    autoChooserUpdate();
    m_autoTab.add("Auto Chooser", m_autoCommand);
    m_autoTab.add("Preloaded Chooser", m_preloadedGamePiece);

    loadCommandSchedulerShuffleboard();
    
    addTestCommands();
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
    m_testTab.add("Cancel Command", new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()));

    if (m_drive != null) {
      m_drive.addTestCommands(m_testTab, testEntry);
    }

    if (m_vision != null) {
      m_vision.addTestCommands(m_testTab, testEntry, m_drive);
    }
  }

  /**
   * Updates the auto chooser on shuffleboard to display what auto routines can be selected for running.
   * 
   * Do Nothing should stay the default, other autos are added with m_autoCommand.addOption()
   */
  public void autoChooserUpdate() {

    Position autoDepositPos = Position.TOP;

    m_preloadedGamePiece.addOption("Cone", GamePieceType.CONE);
    m_preloadedGamePiece.addOption("Cube", GamePieceType.CUBE);

    // add commands below with: m_autoCommand.addOption("Example", new ExampleCommand());
    m_autoCommand.setDefaultOption("Do Nothing", new PrintCommand("This will do nothing!"));

    if (m_drive != null) {
      m_autoCommand.addOption("Figure 8", new PathPlannerCommand("Figure 8", 0, m_drive, true));
      m_autoCommand.addOption("One Meter", new PathPlannerCommand("One Meter", 0, m_drive, true));
    }

    if (m_drive != null && m_elevator != null && m_arm != null && m_intake != null) {

      //TODO: set auto game piece on start

      m_autoCommand.addOption("Hybrid Score", new PositionIntake(m_elevator, m_arm, () -> true, Position.BOTTOM).andThen(new OuttakeGamePiece(m_intake, GamePieceType.CONE)).andThen(new PositionIntake(m_elevator, m_arm, () -> true, Position.STOW)));

      // m_autoCommand.addOption("HYBRID MOBILITY", 
      //   new PositionIntake(m_elevator, m_arm, ()->true, Position.BOTTOM).andThen(
      //   new PathPlannerCommand("Grid 1 Mobility", 0, m_drive, true).andThen(
      //   new Outtake(m_intake).withTimeout(4).andThen(
      //   new Stow(m_intake, m_elevator, m_arm).andThen(
      //     new PathPlannerCommand("Grid 1 Mobility", 1, m_drive)
      //   )))
      // ));

      // TODO: Change the boolean supplier
      // TODO: add hybrid score
      m_autoCommand.addOption("Grid 1 Mobility Top", new DepositThenPath("Grid 1 Mobility", Position.TOP, m_drive, m_elevator, m_arm, m_intake).andThen(new IntakeGamePiece(m_intake, () -> false)));
      m_autoCommand.addOption("Grid 1 Mobility Mid", new DepositThenPath("Grid 1 Mobility", Position.MIDDLE, m_drive, m_elevator, m_arm, m_intake).andThen(new IntakeGamePiece(m_intake, () -> false)));
      // m_autoCommand.addOption("Grid 1 Mobility Hybrid", new DepositThenPath("Grid 1 Mobility", Position.BOTTOM, m_drive, m_elevator, m_arm, m_intake).andThen(new IntakeGamePiece(m_intake, () -> false)));
      m_autoCommand.addOption("Grid 9 Mobility Top", new DepositThenPath("Grid 9 Mobility", Position.TOP, m_drive, m_elevator, m_arm, m_intake).andThen(new IntakeGamePiece(m_intake, () -> false)));
      m_autoCommand.addOption("Grid 9 Mobility Mid", new DepositThenPath("Grid 9 Mobility", Position.MIDDLE, m_drive, m_elevator, m_arm, m_intake).andThen(new IntakeGamePiece(m_intake, () -> false)));
      // m_autoCommand.addOption("Grid 9 Mobility Hybrid", new DepositThenPath("Grid 9 Mobility", Position.BOTTOM, m_drive, m_elevator, m_arm, m_intake).andThen(new IntakeGamePiece(m_intake, () -> false)));
    
      m_autoCommand.addOption("Grid 1 Engage Top", new DepositThenPath("Grid 1 Engage", Position.TOP, m_drive, m_elevator, m_arm, m_intake).andThen(new BalanceCommand(m_drive)));
      m_autoCommand.addOption("Grid 1 Engage Mid", new DepositThenPath("Grid 1 Engage", Position.MIDDLE, m_drive, m_elevator, m_arm, m_intake).andThen(new BalanceCommand(m_drive)));
      // m_autoCommand.addOption("Grid 1 Engage Hybrid", new DepositThenPath("Grid 1 Engage", Position.BOTTOM, m_drive, m_elevator, m_arm, m_intake).andThen(new BalanceCommand(m_drive)));
      m_autoCommand.addOption("UNTESTED Grid 9 Engage Top", new DepositThenPath("Grid 9 Engage", Position.TOP, m_drive, m_elevator, m_arm, m_intake).andThen(new BalanceCommand(m_drive)));
      m_autoCommand.addOption("UNTESTED Grid 9 Engage Mid", new DepositThenPath("Grid 9 Engage", Position.MIDDLE, m_drive, m_elevator, m_arm, m_intake).andThen(new BalanceCommand(m_drive)));
      // m_autoCommand.addOption("UNTESTED Grid 9 Engage Hybrid", new DepositThenPath("Grid 9 Engage", Position.BOTTOM, m_drive, m_elevator, m_arm, m_intake).andThen(new BalanceCommand(m_drive)));
      
      m_autoCommand.addOption("Grid 4/6 Engage Top", new DepositThenPath("Grid 6 Engage No Mobility", Position.TOP, m_drive, m_elevator, m_arm, m_intake).andThen(new BalanceCommand(m_drive)));
      m_autoCommand.addOption("Grid 4/6 Engage Mid", new DepositThenPath("Grid 6 Engage No Mobility", Position.MIDDLE, m_drive, m_elevator, m_arm, m_intake).andThen(new BalanceCommand(m_drive)));
      // m_autoCommand.addOption("Grid 4/6 Engage Hybrid", new DepositThenPath("Grid 6 Engage No Mobility", Position.BOTTOM, m_drive, m_elevator, m_arm, m_intake).andThen(new BalanceCommand(m_drive)));
      
      // m_autoCommand.addOption("NO DEPOSIT Grid 6 Engage (no mobility)",
      //   new PathPlannerCommand("Grid 6 Engage No Mobility", 0, m_drive, true).andThen(
      //   new PathPlannerCommand("Grid 6 Engage No Mobility", 1, m_drive, true)).andThen(
      //   new BalanceCommand(m_drive))
      // );

      // m_autoCommand.addOption("NO DEPOSIT Grid 1 Engage",
      //   new PathPlannerCommand("Grid 1 Engage", 0, m_drive, true).andThen(
      //   new PathPlannerCommand("Grid 1 Engage", 1, m_drive, true)).andThen(
      //   new BalanceCommand(m_drive))
      // );
    }
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

  /**
   * Sets the held game piece type for the intake.
   * @param gamePiece the type of game piece
   */
  public void updateHeldGamePiece() {
    m_intake.setHeldGamePiece(m_preloadedGamePiece.getSelected());
  }

}