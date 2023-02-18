package frc.robot.controls;

import frc.robot.constants.OIConstants;
import lib.controllers.GameController;
public class Operator {

  private GameController operator = new GameController(OIConstants.kOperatorJoy);
  
  /**
   * Configures all of the operator controls.
   */
  public Operator() {

  }

}
