package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DoNothing;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.ShuffleboardManager;
import frc.robot.util.TestType;
import lib.controllers.GameController;
import lib.controllers.GameController.Button;

public class Driver {
  private static GameController driver = new GameController(Constants.oi.kDriverJoy);

  /**
   * Configures all the driver controls, which are the default controls for the robot.
   */
  public static void configureControls(ShuffleboardManager shuffleboard, Drivetrain drive) {
    
    // example button binding implementation
    driver.get(Button.A).onTrue(new DoNothing());

    // example test type implementation
    // tests drivetrain, when in TEST_DRIVE test mode and 
    driver.get(Button.A).and(shuffleboard.isTestTypeTrigger(TestType.TEST_DRIVE)).onTrue(
      new SequentialCommandGroup(
        new RunCommand(() -> drive.tankDrive(0.5, 0.5), drive).withTimeout(1),
        new RunCommand(() -> drive.tankDrive(-0.5, -0.5), drive).withTimeout(1),
        new RunCommand(() -> drive.tankDrive(0.5, -0.5), drive).withTimeout(1),
        new RunCommand(() -> drive.tankDrive(-0.5, 0.5), drive).withTimeout(1)
      )
    );
  }
}
