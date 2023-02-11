package frc.robot.controls;

import frc.robot.commands.DoNothing;
import frc.robot.constants.OIConstants;
import lib.controllers.GameController;
import lib.controllers.GameController.GCButton;

public class Operator {
  private GameController operator = new GameController(OIConstants.kOperatorJoy);

  /**
   * Configures all of the operator controls.
   */
  public void configureControls() {
    operator.get(GCButton.A).onTrue(new DoNothing());
  }
}
