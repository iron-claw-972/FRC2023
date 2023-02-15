package frc.robot.controls;

import frc.robot.commands.arm.ExtendToPosition;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.OIConstants;
import frc.robot.subsystems.FourBarArm;
import lib.controllers.GameController;
import lib.controllers.GameController.Button;

public class Operator {
  private static GameController operator = new GameController(OIConstants.kOperatorJoy);
  
  private static double speed = 0.1;

  /**
   * Configures all of the operator controls.
   */
  public static void configureControls(FourBarArm arm) {
    operator.get(Button.Y).onTrue(new ExtendToPosition(arm, ArmConstants.topPosition));
    operator.get(Button.X).onTrue(new ExtendToPosition(arm, ArmConstants.middlePosiiton));
    operator.get(Button.A).onTrue(new ExtendToPosition(arm, ArmConstants.intakePosition));
    operator.get(Button.Y).onTrue(new ExtendToPosition(arm, ArmConstants.shelfPosition));
  }

  

  
}
