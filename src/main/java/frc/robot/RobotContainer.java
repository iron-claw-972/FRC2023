package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot.RobotId;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.auto.AutoDeposit;
import frc.robot.commands.auto.BalanceCommand;
import frc.robot.commands.auto.PathPlannerCommand;
import frc.robot.commands.auto.TwoPiece;
import frc.robot.commands.scoring.PositionIntake;
import frc.robot.commands.scoring.Stow;
import frc.robot.commands.scoring.PositionIntake.Position;
import frc.robot.commands.scoring.elevator.MoveElevator;
import frc.robot.commands.scoring.intake.IntakeGamePiece;
import frc.robot.commands.scoring.intake.OuttakeGamePiece;
import frc.robot.constants.Constants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.constants.swerve.DriveConstants;
import frc.robot.controls.BaseDriverConfig;
import frc.robot.controls.GameControllerDriverConfig;
import frc.robot.controls.Operator;
import frc.robot.controls.PS5ControllerDriverConfig;
import frc.robot.controls.TestController;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;
import frc.robot.util.Blinkin;
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
  private final ShuffleboardTab m_wristTab = Shuffleboard.getTab("Wrist");

  private final Vision m_vision;

  // The robot's subsystems are defined here...
  private final Drivetrain m_drive;
  private final Intake m_intake;
  private final Elevator m_elevator;
  private final Wrist m_wrist;

  // Controllers are defined here
  private final BaseDriverConfig m_driver;
  private final Operator m_operator;
  private final TestController m_testController;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer(RobotId robotId) {

    // PowerDistribution m_PDModule = new PowerDistribution(1, ModuleType.kRev);
    // m_PDModule.clearStickyFaults();
    // m_PDModule.close();

    switch (robotId) {
      case SwerveCompetition:
        // Update drive constants based off of robot type
        DriveConstants.update(robotId);
        VisionConstants.update(robotId);

        m_vision = new Vision(m_visionTab, VisionConstants.kCameras);

        // Create Drivetrain
        m_drive = new Drivetrain(m_drivetrainTab, m_swerveModulesTab, m_vision);

        m_intake = new Intake(m_intakeTab);
        m_elevator = new Elevator(m_elevatorTab, () -> m_intake.containsGamePiece());
        m_wrist = new Wrist(m_wristTab);

        m_testController = new TestController(m_wrist, m_intake, m_elevator);
  
        m_operator = new Operator();

        DoubleSupplier intakeDist = () -> m_intake.getConePos() * (DriverStation.getAlliance() == Alliance.Blue ? 1 : -1);
        m_driver = new PS5ControllerDriverConfig(m_drive, intakeDist, m_operator, m_controllerTab, false);
        m_operator.configureControls(m_wrist, m_intake, m_elevator, m_vision);
        // m_testController.configureControls();
        // m_manualController.configureControls();
  
        // load paths before auto starts
        PathGroupLoader.loadPathGroups();

        m_driver.configureControls();

        m_vision.setupVisionShuffleboard();
        m_driver.setupShuffleboard();
        m_operator.setUpShuffleboard(m_controllerTab);

        m_drive.setDefaultCommand(new DefaultDriveCommand(m_drive, m_driver));

        break;

      case SwerveTest:
        // Update drive constants based off of robot type
        DriveConstants.update(robotId);
        VisionConstants.update(robotId);

        m_vision = new Vision(m_visionTab, VisionConstants.kCameras);

        // Create Drivetrain, because every robot will have a drivetrain
        m_drive = new Drivetrain(m_drivetrainTab, m_swerveModulesTab, m_vision);
        m_operator = new Operator();
        m_driver = new GameControllerDriverConfig(m_drive, () -> 0, m_operator, m_controllerTab, false);

        DriverStation.reportWarning("Not registering subsystems and controls due to incorrect robot", false);

        // TODO: construct dummy subsystems so SwerveTest can run all auto routines
        m_intake = null;
        m_elevator = null;
        m_wrist = null;
  
        m_testController = null;

        // load paths before auto starts
        PathGroupLoader.loadPathGroups();

        m_driver.configureControls();

        m_vision.setupVisionShuffleboard();
        m_driver.setupShuffleboard();
        m_operator.setUpShuffleboard(m_controllerTab);

        m_drive.setDefaultCommand(new DefaultDriveCommand(m_drive, m_driver));
        
        break;

      default:
        DriverStation.reportWarning("Not registering subsystems and controls due to incorrect robot", false);

        m_vision = null;

        m_driver = null;
        m_drive = null;

        m_intake = null;
        m_elevator = null;
        m_wrist = null;

        m_operator = null;
        m_testController = null;
        break;
    }

    // This is really annoying so it's disabled
    DriverStation.silenceJoystickConnectionWarning(true);

    LiveWindow.disableAllTelemetry(); // LiveWindow is causing periodic loop overruns
    LiveWindow.setEnabled(false);
    
    autoChooserUpdate();
    m_autoTab.add("Auto Chooser", m_autoCommand);

    if (Constants.kUseTelemetry) loadCommandSchedulerShuffleboard();
    
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
    GenericEntry blinkinId = m_testTab.add("Blinkin Id",0.65).getEntry();
    m_testTab.add("Cancel Command", new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()));
    m_testTab.add("Color", new InstantCommand( () -> Blinkin.setColor(blinkinId.getDouble(0.65))));

    if (m_drive != null) {
      m_drive.addTestCommands(m_testTab, testEntry);
      m_testTab.add("Reset Odometry to node", new InstantCommand(() -> m_drive.resetOdometry(m_operator.getSelectedNode().scorePose)));
      m_testTab.add("Reset Odometry to Blue Shelf", new InstantCommand(() -> m_drive.resetOdometry(VisionConstants.kBlueShelfAlignPose)));
      m_testTab.add("Reset Odometry to Red Shelf", new InstantCommand(() -> m_drive.resetOdometry(VisionConstants.kRedShelfAlignPose)));
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

    m_autoCommand.setDefaultOption("Do Nothing", new PrintCommand("This will do nothing!"));

    // if (m_drive != null) {
    //   m_autoCommand.addOption("Figure 8", new PathPlannerCommand("Figure 8", 0, m_drive, true));
    //   m_autoCommand.addOption("One Meter", new PathPlannerCommand("One Meter", 0, m_drive, true));
    // }

    if (m_drive != null && m_elevator != null && m_wrist != null && m_intake != null) {

      // m_autoCommand.addOption("ROUTINE 1: Grid 4 Engage Hybrid", Commands.sequence(
      //     new AutoDeposit(Position.BOTTOM, true, m_elevator, m_wrist, m_intake),
      //     new PathPlannerCommand("Grid 4 Engage No Mobility", 0, m_drive, true),
      //     new BalanceCommand(m_drive)));
      // m_autoCommand.addOption("ROUTINE 2: Grid 4 Engage Mid", Commands.sequence(
      //     new AutoDeposit(Position.MIDDLE, true, m_elevator, m_wrist, m_intake),
      //     new PathPlannerCommand("Grid 4 Engage No Mobility", 0, m_drive, true),
      //     new BalanceCommand(m_drive)));
      m_autoCommand.addOption("ROUTINE 3: Grid 4 Engage Top", Commands.sequence(
          new AutoDeposit(Position.TOP, true, m_elevator, m_wrist, m_intake),
          new PathPlannerCommand("Grid 4 Engage No Mobility", 0, m_drive, true),
          new BalanceCommand(m_drive)));

      // m_autoCommand.addOption("ROUTINE 4: Grid 6 Engage Hybrid", Commands.sequence(
      //     new AutoDeposit(Position.BOTTOM, true, m_elevator, m_wrist, m_intake),
      //     new PathPlannerCommand("Grid 6 Engage No Mobility", 0, m_drive, true),
      //     new BalanceCommand(m_drive)));
      // m_autoCommand.addOption("ROUTINE 5: Grid 6 Engage Mid", Commands.sequence(
      //     new AutoDeposit(Position.MIDDLE, true, m_elevator, m_wrist, m_intake),
      //     new PathPlannerCommand("Grid 6 Engage No Mobility", 0, m_drive, true),
      //     new BalanceCommand(m_drive)));
      m_autoCommand.addOption("ROUTINE 6: Grid 6 Engage Top", Commands.sequence(
          new AutoDeposit(Position.TOP, true, m_elevator, m_wrist, m_intake),
          new PathPlannerCommand("Grid 6 Engage No Mobility", 0, m_drive, true),
          new BalanceCommand(m_drive)));

      // m_autoCommand.addOption("ROUTINE 7: Grid 1 Mobility Hybrid", Commands.sequence(
      //     new AutoDeposit(Position.BOTTOM, true, m_elevator, m_wrist, m_intake),
      //     new PathPlannerCommand("Grid 1 Mobility", 0, m_drive, true)));
      // m_autoCommand.addOption("ROUTINE 8: Grid 1 Mobility Mid", Commands.sequence(
      //     new AutoDeposit(Position.MIDDLE, true, m_elevator, m_wrist, m_intake),
      //     new PathPlannerCommand("Grid 1 Mobility", 0, m_drive, true)));
      m_autoCommand.addOption("ROUTINE 9: Grid 1 Mobility Top", Commands.sequence(
          new AutoDeposit(Position.TOP, true, m_elevator, m_wrist, m_intake),
          new PathPlannerCommand("Grid 1 Mobility", 0, m_drive, true)));

      // m_autoCommand.addOption("ROUTINE 10: Grid 9 Mobility Hybrid", Commands.sequence(
      //     new AutoDeposit(Position.BOTTOM, true, m_elevator, m_wrist, m_intake),
      //     new PathPlannerCommand("Grid 9 Mobility", 0, m_drive, true)));
      // m_autoCommand.addOption("ROUTINE 11: Grid 9 Mobility Mid", Commands.sequence(
      //     new AutoDeposit(Position.MIDDLE, true, m_elevator, m_wrist, m_intake),
      //     new PathPlannerCommand("Grid 9 Mobility", 0, m_drive, true)));
      m_autoCommand.addOption("ROUTINE 12: Grid 9 Mobility Top", Commands.sequence(
          new AutoDeposit(Position.TOP, true, m_elevator, m_wrist, m_intake),
          new PathPlannerCommand("Grid 9 Mobility", 0, m_drive, true)));

      // m_autoCommand.addOption("ROUTINE 13: Grid 1 Engage Hybrid", Commands.sequence(
      //     new AutoDeposit(Position.BOTTOM, true, m_elevator, m_wrist, m_intake),
      //     new PathPlannerCommand("Grid 1 Engage", 0, m_drive, true),
      //     new BalanceCommand(m_drive)));
      // m_autoCommand.addOption("ROUTINE 14: Grid 1 Engage Mid", Commands.sequence(
      //     new AutoDeposit(Position.MIDDLE, true, m_elevator, m_wrist, m_intake),
      //     new PathPlannerCommand("Grid 1 Engage", 0, m_drive, true),
      //     new BalanceCommand(m_drive)));
      m_autoCommand.addOption("ROUTINE 15: Grid 1 Engage Top", Commands.sequence(
          new AutoDeposit(Position.TOP, true, m_elevator, m_wrist, m_intake),
          new PathPlannerCommand("Grid 1 Engage", 0, m_drive, true),
          new BalanceCommand(m_drive)));

      // m_autoCommand.addOption("ROUTINE 16: Grid 9 Engage Hybrid", Commands.sequence(
      //     new AutoDeposit(Position.BOTTOM, true, m_elevator, m_wrist, m_intake),
      //     new PathPlannerCommand("Grid 9 Engage", 0, m_drive, true),
      //     new BalanceCommand(m_drive)));
      // m_autoCommand.addOption("ROUTINE 17: Grid 9 Engage Mid", Commands.sequence(
      //     new AutoDeposit(Position.MIDDLE, true, m_elevator, m_wrist, m_intake),
      //     new PathPlannerCommand("Grid 9 Engage", 0, m_drive, true),
      //     new BalanceCommand(m_drive)));
      m_autoCommand.addOption("ROUTINE 18: Grid 9 Engage Top", Commands.sequence(
          new AutoDeposit(Position.TOP, true, m_elevator, m_wrist, m_intake),
          new PathPlannerCommand("Grid 9 Engage", 0, m_drive, true),
          new BalanceCommand(m_drive)));

      // m_autoCommand.addOption("ROUTINE 19: Grid 1 Intake Cone Top",
      //     Commands.sequence(
      //         new AutoDeposit(Position.TOP, true, m_elevator, m_wrist, m_intake),
      //         new PathPlannerCommand("Grid 1 Two Piece", 0, m_drive, true)
      //             .alongWith(new WaitCommand(1).andThen(
      //               new PositionIntake(m_elevator, m_wrist, GamePieceType.CONE, Position.INTAKE), 
      //               new IntakeGamePiece(m_intake, GamePieceType.CONE, false)))));

      m_autoCommand.addOption("ROUTINE 20: Grid 1 Intake Cube Top",
          Commands.sequence(
              new AutoDeposit(Position.TOP, true, m_elevator, m_wrist, m_intake),
              new PathPlannerCommand("Grid 1 Two Piece", 0, m_drive, true)
                  .alongWith(new WaitCommand(1).andThen(
                    new PositionIntake(m_elevator, m_wrist, GamePieceType.CUBE, Position.INTAKE), 
                    new IntakeGamePiece(m_intake, GamePieceType.CUBE, false)))));

      // m_autoCommand.addOption("ROUTINE 21: Grid 9 Intake Cone Top",
      //     Commands.sequence(
      //         new AutoDeposit(Position.TOP, true, m_elevator, m_wrist, m_intake),
      //         new PathPlannerCommand("Grid 9 Two Piece", 0, m_drive, true)
      //           .alongWith(new WaitCommand(1.5).andThen(
      //             new PositionIntake(m_elevator, m_wrist, GamePieceType.CONE, Position.INTAKE), 
      //             new IntakeGamePiece(m_intake, GamePieceType.CONE, false)))));
      
      m_autoCommand.addOption("ROUTINE 22: Grid 9 Intake Cube Top",
          Commands.sequence(
              new AutoDeposit(Position.TOP, true, m_elevator, m_wrist, m_intake),
              new PathPlannerCommand("Grid 9 Two Piece", 0, m_drive, true)
                .alongWith(new WaitCommand(1.5).andThen(
                  new PositionIntake(m_elevator, m_wrist, GamePieceType.CUBE, Position.INTAKE), 
                  new IntakeGamePiece(m_intake, GamePieceType.CUBE, false)))));

      m_autoCommand.addOption("ROUTINE 23: Grid 1 Two Piece Cube Top",
          Commands.sequence(
            new TwoPiece(false, m_drive, m_elevator, m_wrist, m_intake),
            new Stow(m_elevator, m_wrist) 
          ));

      m_autoCommand.addOption("ROUTINE 24: Grid 9 Two Piece Cube Top",
          Commands.sequence(
            new TwoPiece(true, m_drive, m_elevator, m_wrist, m_intake),
            new Stow(m_elevator, m_wrist)
          ));

      m_autoCommand.addOption("ROUTINE 25: Grid 1 Two Piece Cube Top Engage",
          Commands.sequence(
              new TwoPiece(false, m_drive, m_elevator, m_wrist, m_intake),
              new PathPlannerCommand("Grid 1 Two Piece Engage", 0, m_drive, true)
                .alongWith(new Stow(m_elevator, m_wrist)),
              new BalanceCommand(m_drive)));
      
      m_autoCommand.addOption("ROUTINE 26: Grid 9 Two Piece Cube Top Engage",
          Commands.sequence(
              new TwoPiece(true, m_drive, m_elevator, m_wrist, m_intake),
              new PathPlannerCommand("Grid 9 Two Piece Engage", 0, m_drive, true)
                .alongWith(new Stow(m_elevator, m_wrist)),
              new BalanceCommand(m_drive)));

      m_autoCommand.addOption("Routine 27: Grid 4/6 Engage Mobility", Commands.sequence(
        new AutoDeposit(Position.TOP, true, m_elevator, m_wrist, m_intake),
        new PathPlannerCommand("Grid 6 Engage Mobility", 0, m_drive, true),
        new BalanceCommand(m_drive)));
      
      // THREE PIECE ROUTINES
      m_autoCommand.addOption("Routine 28: Grid 1 Three Piece", 
          Commands.sequence(
              new TwoPiece(false, m_drive, m_elevator, m_wrist, m_intake),
              new PathPlannerCommand("Grid 1 Three Piece", 0, m_drive, false)
                .alongWith(Commands.sequence(
                  new Stow(m_elevator, m_wrist),
                  new WaitCommand(0.75),
                  new PositionIntake(m_elevator, m_wrist, GamePieceType.CUBE, Position.INTAKE)
                    .alongWith(new IntakeGamePiece(m_intake, GamePieceType.CUBE, false))
                )),
              new PathPlannerCommand("Grid 1 Three Piece", 1, m_drive, false)
                .alongWith(Commands.sequence(
                  new Stow(m_elevator, m_wrist),
                  new WaitCommand(2),
                  new OuttakeGamePiece(m_intake, () -> GamePieceType.CUBE)
                ))
          ));

      m_autoCommand.addOption("Routine 29: Grid 9 Three Piece",
          Commands.sequence(
            new TwoPiece(true, m_drive, m_elevator, m_wrist, m_intake),
            new PathPlannerCommand("Grid 9 Three Piece", 0, m_drive, false)
              .alongWith(Commands.sequence( 
                  new Stow(m_elevator, m_wrist), 
                  new WaitCommand(0.75),
                  new PositionIntake(m_elevator, m_wrist, GamePieceType.CUBE, Position.INTAKE)
                    .alongWith(new IntakeGamePiece(m_intake, GamePieceType.CUBE, false))
              )),
            new PathPlannerCommand("Grid 9 Three Piece", 1, m_drive, false)
              .alongWith(Commands.sequence(
                new Stow(m_elevator, m_wrist),
                new WaitCommand(2),
                new OuttakeGamePiece(m_intake, () -> GamePieceType.CUBE)
              ))
          ));

        // m_autoCommand.addOption("Routine 30: Grid 9 Two Piece Intake 1",
        //   Commands.sequence(
        //     new TwoPiece(true, m_drive, m_elevator, m_wrist, m_intake),
        //     new PathPlannerCommand("Grid 9 Three Piece", 0, m_drive, false)
        //       .alongWith(Commands.sequence( 
        //           new Stow(m_elevator, m_wrist), 
        //           new WaitCommand(0.75),
        //           new PositionIntake(m_elevator, m_wrist, GamePieceType.CUBE, Position.INTAKE)
        //             .alongWith(new IntakeGamePiece(m_intake, GamePieceType.CUBE, false))
        //       )),
        //     new PathPlannerCommand("Grid 9 Three Piece", 1, m_drive, false)
        //       .alongWith(new Stow(m_elevator, m_wrist))
        //   ));
        
        m_autoCommand.addOption("Routine 31: Grid 4/6 Engage with Intake", Commands.sequence(
          new AutoDeposit(Position.TOP, false, m_elevator, m_wrist, m_intake),
          new PathPlannerCommand("Grid 6 Engage Intake", 0, m_drive, true)
            .deadlineWith(Commands.sequence(
              new Stow(m_elevator, m_wrist),
              new WaitCommand(2.2),
              new PositionIntake(m_elevator, m_wrist, GamePieceType.CUBE, Position.INTAKE)
                .alongWith(new IntakeGamePiece(m_intake, GamePieceType.CUBE, false)),
              new Stow(m_elevator, m_wrist)
            )),
          new AutoDeposit(Position.BOTTOM, GamePieceType.CUBE, true, m_elevator, m_wrist, m_intake),
          new BalanceCommand(m_drive, 0.2)
        ));
        

        // m_autoCommand.addOption("Routine 30: Grid 9 Three Piece Hybrid",
        //   Commands.sequence(
        //       new AutoDeposit(Position.BOTTOM, GamePieceType.CONE, true, m_elevator, m_wrist, m_intake),
        //       new PathPlannerCommand("Grid 9 Three Piece Hybrid", 0, m_drive, true)
        //         .alongWith(new WaitCommand(1).andThen(Commands.parallel(
        //           new PositionIntake(m_elevator, m_wrist, GamePieceType.CUBE, Position.INTAKE), 
        //           new IntakeGamePiece(m_intake, GamePieceType.CUBE, false)))),
        //       new PathPlannerCommand("Grid 9 Three Piece Hybrid", 1, m_drive, false).alongWith(
        //         new Stow(m_elevator, m_wrist)),
        //       new AutoDeposit(Position.BOTTOM, GamePieceType.CUBE, true, m_elevator, m_wrist, m_intake),
        //       new PathPlannerCommand("Grid 9 Three Piece Hybrid", 2, m_drive, false)
        //         .alongWith(new WaitCommand(1).andThen(
        //           new PositionIntake(m_elevator, m_wrist, GamePieceType.CUBE, Position.INTAKE), 
        //           new IntakeGamePiece(m_intake, GamePieceType.CUBE, false))),
        //       new PathPlannerCommand("Grid 9 Three Piece Hybrid", 3, m_drive, false).alongWith(
        //         new Stow(m_elevator, m_wrist)),
        //       new AutoDeposit(Position.BOTTOM, GamePieceType.CUBE, true, m_elevator, m_wrist, m_intake)));
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
    if(m_intake != null)
    {
      m_intake.setHeldGamePiece(GamePieceType.CONE);
    }
  }

  /**
   * Resets the swerve modules to their absolute positions.
   */
  public void resetModules() {
    m_drive.resetModulesToAbsolute();
  }

  /**
   * Sets whether or not the drivetrain uses vision to update odometry
   */
  public void setVisionEnabled(boolean enabled) {
    m_drive.enableVision(enabled);
  }

}