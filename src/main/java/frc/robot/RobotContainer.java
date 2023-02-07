package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.controls.Driver;
import frc.robot.controls.Operator;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.PathGroupLoader;
import frc.robot.util.ShuffleboardManager;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain m_drive = new Drivetrain();
  private final ShuffleboardManager m_shuffleboard;
  private final Robot m_robot;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer(Robot robot) {

    // This is really annoying so it's disabled
    DriverStation.silenceJoystickConnectionWarning(true);
    
    // it may be needed to disable LiveWindow (we don't use it anyway)
    //LiveWindow.setEnabled(false)

    // load paths before auto starts
    PathGroupLoader.loadPathGroups();

    m_robot = robot;

    m_shuffleboard = new ShuffleboardManager(m_robot);

    Driver.configureControls(m_shuffleboard, m_drive);
    Operator.configureControls();

  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_shuffleboard.getAutonomousCommand();
  }
}