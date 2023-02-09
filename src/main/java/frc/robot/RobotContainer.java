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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.controls.Driver;
import frc.robot.controls.Operator;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.PathGroupLoader;
import frc.robot.util.TestType;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems are defined here...
  private final Drivetrain m_drive = new Drivetrain();
  private final Robot m_robot;

  // Shuffleboard stuff
  SendableChooser<Command> m_autoCommand = new SendableChooser<>();
  SendableChooser<TestType> m_testType = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer(Robot robot) {

    // This is really annoying so it's disabled
    DriverStation.silenceJoystickConnectionWarning(true);

    // load paths before auto starts
    PathGroupLoader.loadPathGroups();

    m_robot = robot;

    Driver.configureControls(this, m_drive);
    Operator.configureControls();

    LiveWindow.disableAllTelemetry(); // LiveWindow is causing periodic loop overruns
    LiveWindow.setEnabled(false);
    
    testChooserUpdate();
    autoChooserUpdate();
    loadCommandSchedulerShuffleboard();

    Shuffleboard.getTab("Auto").add("Auto Chooser", m_autoCommand);
    Shuffleboard.getTab("Test").add("Run Test", m_testType);
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autoCommand.getSelected();
  }
  
  public TestType getTestType() {
    if (m_robot.isTest()) {
      return m_testType.getSelected();
    }
    return TestType.NONE;
  }

  public Trigger isTestTypeTrigger(TestType testType) {
    return new Trigger(() -> m_testType.getSelected() == testType);
  }

  public void testChooserUpdate() {
    m_testType.setDefaultOption("No Test", TestType.NONE);
  }

  public void autoChooserUpdate() {
    m_autoCommand.setDefaultOption("Do Nothing", new PrintCommand("This will do nothing!"));
  }

  public void loadCommandSchedulerShuffleboard() {
    // Set the scheduler to log Shuffleboard events for command initialize, interrupt, finish
    CommandScheduler.getInstance().onCommandInitialize(command -> Shuffleboard.addEventMarker("Command initialized", command.getName(), EventImportance.kNormal));
    CommandScheduler.getInstance().onCommandInterrupt(command -> Shuffleboard.addEventMarker("Command interrupted", command.getName(), EventImportance.kNormal));
    CommandScheduler.getInstance().onCommandFinish(command -> Shuffleboard.addEventMarker("Command finished", command.getName(), EventImportance.kNormal));
  }
}