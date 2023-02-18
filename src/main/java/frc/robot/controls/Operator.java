package frc.robot.controls;

import frc.robot.commands.BalanceCommand;
import frc.robot.commands.DoNothing;
import frc.robot.constants.OIConstants;
import frc.robot.subsystems.Drivetrain;
import lib.controllers.GameController;
import lib.controllers.GameController.GCButton;

// TODO
public class Operator {
  private GameController operator = new GameController(OIConstants.kOperatorJoy);

  /**
   * Configures all of the operator controls.
   */
  public void configureControls(Drivetrain drive) {
    operator.get(GCButton.A).onTrue(new DoNothing());
    operator.get(GCButton.RB).onTrue(new BalanceCommand(drive));
  }
}
