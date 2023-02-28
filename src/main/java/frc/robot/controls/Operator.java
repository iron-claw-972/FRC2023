package frc.robot.controls;

import frc.robot.commands.deployingbar.RotateDeployingBar;
import frc.robot.constants.DeployingBarConstants;
import frc.robot.constants.OIConstants;
import frc.robot.subsystems.FourBarArm;
import frc.robot.subsystems.DeployingBar;
import frc.robot.subsystems.Intake;
import lib.controllers.GameController;
import lib.controllers.GameController.Button;

public class Operator {

  private GameController m_operator = new GameController(OIConstants.kOperatorJoy);
  
  /**
   * Configures the operator controls for the deploying Bar.
   */
  public void configureControls(DeployingBar deployingBar) {
    m_operator.get(Button.A).onTrue(new RotateDeployingBar(deployingBar, DeployingBarConstants.kStowRotation));
    m_operator.get(Button.B).onTrue(new RotateDeployingBar(deployingBar, DeployingBarConstants.kDeployedRotation));
  }

  /**
   * Configures the operator controls for the arm.
   */
  public void configureControls(FourBarArm arm) {

  }

  /**
   * Configures the operator controls for the intake.
   */
  public void configureControls(Intake intake) {

  }
}
