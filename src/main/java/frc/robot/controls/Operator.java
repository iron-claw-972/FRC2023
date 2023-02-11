package frc.robot.controls;

import frc.robot.commands.DoNothing;
import frc.robot.constants.Constants;
import frc.robot.constants.OIConstants;
import lib.controllers.GameController;
import lib.controllers.GameController.GCButton;

public class Operator {
  private static GameController operator = new GameController(OIConstants.kOperatorJoy);

  public void configureControls() {
    operator.get(GCButton.A).whenPressed(new DoNothing());
  }

}
