package frc.robot.controls;

import frc.robot.commands.deployingBar.RotateBottomLimitSwitch;
import frc.robot.commands.deployingBar.RotateTopLimitSwitch;
import frc.robot.constants.OIConstants;
import frc.robot.subsystems.FourBarArm;
import frc.robot.subsystems.DeployingBar;
import frc.robot.subsystems.Intake;
import lib.controllers.GameController;
import lib.controllers.GameController.Button;

public class Operator {

  private GameController operator = new GameController(OIConstants.kOperatorJoy);
  private FourBarArm m_arm;
  private Intake m_intake;
  private DeployingBar m_deployingBar;

  public Operator(FourBarArm arm, Intake intake, DeployingBar deployingBar){
      m_arm = arm;
      m_intake = intake;
      m_deployingBar = deployingBar;
  }
  
  /**
   * Configures all of the operator controls.
   */
  public void configureControls() {
    operator.get(Button.A).onTrue(new RotateTopLimitSwitch(m_deployingBar));
    operator.get(Button.B).onTrue(new RotateBottomLimitSwitch(m_deployingBar));

  }
}
