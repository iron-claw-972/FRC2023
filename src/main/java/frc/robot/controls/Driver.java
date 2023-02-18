package frc.robot.controls;

import frc.robot.commands.DoNothing;
import frc.robot.constants.OIConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.FourBarArm;
import lib.controllers.GameController;
import lib.controllers.GameController.Button;

public class Driver {
  private static GameController driver = new GameController(OIConstants.kDriverJoy);

  /**
   * Configures all the driver controls, which are the default controls for the robot.
   */
  public static void configureControls(Drivetrain drive, FourBarArm arm) {
    
    // example button binding implementation
    driver.get(Button.A).onTrue(new DoNothing());
    //TODO: Change this to call whatever function swerve uses
    // driver.get(Button.RB).toggleOnTrue(new MoveToPose(drive, Operator.selectedNode.scorePose));
  }
}
