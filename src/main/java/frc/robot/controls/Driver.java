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
  public static GameController controller = new GameController(Constants.oi.kDriverJoy);

  public static void configureControls() {
    
    // example button binding implementation
    controller.get(Button.A).onTrue(new DoNothing());

    controller.get(Button.X).onTrue(new InstantCommand(() -> {
      Robot.intake.intake();
    }, Robot.intake));

    controller.get(Button.B).onTrue(new InstantCommand(() -> {
      Robot.intake.outtake();
    }, Robot.intake));

    controller.get(Button.Y).onTrue(new InstantCommand(() -> {
      Robot.intake.stop();
    }, Robot.intake));
    
  }
}
