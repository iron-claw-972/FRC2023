package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.DoNothing;
import frc.robot.constants.Constants;
import frc.robot.util.TestType;
import lib.controllers.GameController;
import lib.controllers.GameController.Button;

public class Driver {

  private static double speed = 0.1;

  public static GameController controller = new GameController(Constants.oi.kDriverJoy);

  public static void configureControls() {
    
    // example button binding implementation
    controller.get(Button.A).onTrue(new InstantCommand(() -> {
      if (speed == 0.1) {
        speed = 0.2;
      } else {
        speed = 0.1;
      }
    }, Robot.intake));

    controller.get(Button.X).onTrue(new InstantCommand(() -> {
      Robot.intake.intake(speed);
    }, Robot.intake));

    controller.get(Button.B).onTrue(new InstantCommand(() -> {
      Robot.intake.outtake(speed);
    }, Robot.intake));

    controller.get(Button.Y).onTrue(new InstantCommand(() -> {
      Robot.intake.stop();
    }, Robot.intake));
    
  }
}
