package frc.robot.controls;

import frc.robot.commands.arm.ExtendToPosition;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.OIConstants;
import frc.robot.subsystems.FourBarArm;
import lib.controllers.GameController;
import lib.controllers.GameController.Button;

// TODO: Change operator control system to follow new BaseDriverConfig system
public class Operator {

  private final GameController kOperator = new GameController(OIConstants.kOperatorJoy);

  /**
   * Configures all of the operator controls.
   */
  public void configureControls(FourBarArm arm) {
    kOperator.get(Button.Y).onTrue(new ExtendToPosition(arm, ArmConstants.kTopPosition));
    kOperator.get(Button.X).onTrue(new ExtendToPosition(arm, ArmConstants.kMiddlePosition));
    kOperator.get(Button.A).onTrue(new ExtendToPosition(arm, ArmConstants.kIntakePosition));
    kOperator.get(Button.B).onTrue(new ExtendToPosition(arm, ArmConstants.kShelfPosition));
  }
}
