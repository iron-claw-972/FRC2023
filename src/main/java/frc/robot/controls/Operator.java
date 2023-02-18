package frc.robot.controls;

import frc.robot.commands.RotateDeployingBar;
import frc.robot.constants.DeployingBarConstants;
import frc.robot.constants.OIConstants;
import frc.robot.subsystems.DeployingBar;
import frc.robot.commands.arm.ExtendToPosition;
import frc.robot.constants.ArmConstants;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.arm.ExtendToPosition;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.OIConstants;
import frc.robot.subsystems.FourBarArm;
import frc.robot.subsystems.Intake;
import lib.controllers.GameController;
import lib.controllers.GameController.Button;
import lib.controllers.GameController.DPad;

public class Operator {
  private static GameController operator = new GameController(OIConstants.kOperatorJoy);
   /**
   * Configures all of the operator controls.
   */
  public static void configureControls(DeployingBar deployingBar, FourBarArm arm, Intake intake) {

    operator.get(Button.A).onTrue(new RotateDeployingBar(deployingBar, DeployingBarConstants.kMaxRotation));
    operator.get(Button.B).onTrue(new RotateDeployingBar(deployingBar, DeployingBarConstants.kMinRotation));
    
    // elevator controls
    operator.get(Button.Y).onTrue(new ExtendToPosition(arm, ArmConstants.topPosition));
    operator.get(Button.X).onTrue(new ExtendToPosition(arm, ArmConstants.middlePosition));
    operator.get(Button.A).onTrue(new ExtendToPosition(arm, ArmConstants.intakePosition));
    operator.get(Button.B).onTrue(new ExtendToPosition(arm, ArmConstants.shelfPosition));

    // intake controls
    operator.get(DPad.DOWN).onTrue(new InstantCommand(() -> intake.intake(IntakeConstants.kIntakeSpeed), intake));
    operator.get(DPad.UP).onTrue(new InstantCommand(() -> intake.intake(IntakeConstants.kOuttakeSpeed),intake));
    operator.get(DPad.LEFT).onTrue(new InstantCommand(() -> intake.stop(), intake));
  }
}
