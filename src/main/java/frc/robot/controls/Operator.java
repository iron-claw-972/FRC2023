package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.ExtendToPosition;
import frc.robot.constants.ArmConstants;
import frc.robot.subsystems.FourBarArm;
import lib.controllers.GameController;
import lib.controllers.GameController.Button;

public class Operator {
  private static GameController operator = new GameController(0);

  /**
   * Configures all of the operator controls.
   */
  public static void configureControls(FourBarArm arm) {
    
  }
}
