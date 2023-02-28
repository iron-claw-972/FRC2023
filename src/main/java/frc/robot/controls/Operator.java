package frc.robot.controls;

import frc.robot.constants.OIConstants;
import frc.robot.subsystems.FourBarArm;
import frc.robot.subsystems.Intake;
import lib.controllers.GameController;

public class Operator {

  private GameController m_operator = new GameController(OIConstants.kOperatorJoy);
  private FourBarArm m_arm;
  private Intake m_intake;

  public Operator(FourBarArm arm, Intake intake){
    m_arm = arm;
    m_intake = intake;
  }

  /**
   * Configures all of the operator controls.
   */
  public void configureControls() {

  }
}
