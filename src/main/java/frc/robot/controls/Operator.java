package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.arm.ExtendToPosition;
import frc.robot.commands.arm.Retract;
import frc.robot.constants.Constants;
import lib.controllers.GameController;
import lib.controllers.GameController.Button;

public class Operator {
  private static GameController operator = new GameController(Constants.oi.kOperatorJoy);

  public static void configureControls() {
    operator.get(Button.Y).onTrue(new SequentialCommandGroup(new ExtendToPosition(Robot.arm, Robot.arm.getPIDController(), Constants.arm.topPosition), new Retract(Robot.arm, Robot.arm.getPIDController())));
    operator.get(Button.X).onTrue(new SequentialCommandGroup(new ExtendToPosition(Robot.arm, Robot.arm.getPIDController(), Constants.arm.middlePosiiton), new Retract(Robot.arm, Robot.arm.getPIDController())));
    operator.get(Button.A).onTrue(new SequentialCommandGroup(new ExtendToPosition(Robot.arm, Robot.arm.getPIDController(), Constants.arm.intakePosition), new Retract(Robot.arm, Robot.arm.getPIDController())));
    operator.get(Button.Y).onTrue(new SequentialCommandGroup(new ExtendToPosition(Robot.arm, Robot.arm.getPIDController(), Constants.arm.shelfPosition), new Retract(Robot.arm, Robot.arm.getPIDController())));
  }
}
