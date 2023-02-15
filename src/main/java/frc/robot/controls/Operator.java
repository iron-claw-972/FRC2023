package frc.robot.controls;

import frc.robot.commands.ExtendDeployingBar;
import frc.robot.constants.DeployingBarConstants;
import frc.robot.constants.OIConstants;
import frc.robot.subsystems.DeployingBar;
import lib.controllers.GameController;
import lib.controllers.GameController.Button;

public class Operator {
  private static GameController operator = new GameController(OIConstants.kOperatorJoy);
   /**
   * Configures all of the operator controls.
   */
  public static void configureControls(DeployingBar deployingBar) {
    operator.get(Button.A).onTrue(new ExtendDeployingBar(deployingBar, DeployingBarConstants.kMaxExtension));
    operator.get(Button.B).onTrue(new ExtendDeployingBar(deployingBar, DeployingBarConstants.kMinExtension));
   
  }
}
