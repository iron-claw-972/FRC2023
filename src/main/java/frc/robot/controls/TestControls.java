package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.OIConstants;
import frc.robot.subsystems.Drivetrain;
import lib.controllers.GameController;
import lib.controllers.GameController.Button;

public class TestControls {
    private static GameController controller = new GameController(OIConstants.kTestJoy);

    public static void configureControls(Drivetrain drive) {

        // example test type implementation
        // tests drivetrain
        controller.get(Button.A).onTrue(
          new SequentialCommandGroup(
            new RunCommand(() -> drive.tankDrive(0.5, 0.5), drive).withTimeout(1),
            new RunCommand(() -> drive.tankDrive(-0.5, -0.5), drive).withTimeout(1),
            new RunCommand(() -> drive.tankDrive(0.5, -0.5), drive).withTimeout(1),
            new RunCommand(() -> drive.tankDrive(-0.5, 0.5), drive).withTimeout(1)
          )
        );
    }
}