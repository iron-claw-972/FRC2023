package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.ExtendToPosition;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.OIConstants;
import frc.robot.subsystems.FourBarArm;
import lib.controllers.GameController;
import lib.controllers.GameController.Button;

public class Operator {
  private static GameController operator = new GameController(OIConstants.kOperatorJoy);

  /**
   * Configures all of the operator controls.
   */
  public static void configureControls(FourBarArm arm) {
    operator.get(Button.Y).onTrue(new SequentialCommandGroup(new ExtendToPosition(arm, ArmConstants.topPosition), new ExtendToPosition(arm, ArmConstants.initialPosition)));
    operator.get(Button.X).onTrue(new SequentialCommandGroup(new ExtendToPosition(arm, ArmConstants.middlePosiiton), new ExtendToPosition(arm, ArmConstants.initialPosition)));
    operator.get(Button.A).onTrue(new SequentialCommandGroup(new ExtendToPosition(arm, ArmConstants.intakePosition), new ExtendToPosition(arm, ArmConstants.initialPosition)));
    operator.get(Button.Y).onTrue(new SequentialCommandGroup(new ExtendToPosition(arm, ArmConstants.shelfPosition), new ExtendToPosition(arm, ArmConstants.initialPosition)));
  }
}
