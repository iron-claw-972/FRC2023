package frc.robot.controls;

import frc.robot.commands.arm.ExtendToPosition;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.OIConstants;
import frc.robot.subsystems.FourBarArm;
import lib.controllers.GameController;
import lib.controllers.GameController.Button;

public class Operator {
  private GameController operator = new GameController(OIConstants.kOperatorJoy);

  /**
   * Configures all of the operator controls.
   */
  public void configureControls(FourBarArm arm) {
    operator.get(Button.Y).onTrue(new ExtendToPosition(arm, ArmConstants.topPosition));
    operator.get(Button.X).onTrue(new ExtendToPosition(arm, ArmConstants.middlePosition));
    operator.get(Button.A).onTrue(new ExtendToPosition(arm, ArmConstants.intakePosition));
    operator.get(Button.B).onTrue(new ExtendToPosition(arm, ArmConstants.shelfPosition));
  }
}
