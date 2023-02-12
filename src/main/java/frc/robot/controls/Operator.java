package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.ExtendDeployingBar;
import frc.robot.constants.Constants;
import frc.robot.constants.DeployingBarConstants;
import frc.robot.subsystems.DeployingBar;
import lib.controllers.GameController;
import lib.controllers.GameController.Button;

public class Operator {
  private static GameController operator = new GameController(Constants.oi.kOperatorJoy);

  public static void configureControls(DeployingBar deployingbar) {
    operator.get(Button.A).onTrue(new ExtendDeployingBar(deployingbar, DeployingBarConstants.kmaxExtension));
    operator.get(Button.B).onTrue(new ExtendDeployingBar(deployingbar, DeployingBarConstants.kminExtension));
   
  }
}
