package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.arm.ExtendToPosition;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.OIConstants;
import frc.robot.subsystems.FourBarArm;
import frc.robot.subsystems.Intake;
import lib.controllers.GameController;
import lib.controllers.GameController.Button;

public class Operator {
  private static GameController operator = new GameController(OIConstants.kOperatorJoy);

  /**
   * Configures all of the operator controls.
   */
  public static void configureControls(FourBarArm arm, Intake intake) {
    // operator.get(Button.Y).onTrue(new ExtendToPosition(arm, ArmConstants.topPosition));
    // operator.get(Button.X).onTrue(new ExtendToPosition(arm, ArmConstants.middlePosiiton));
    // operator.get(Button.A).onTrue(new ExtendToPosition(arm, ArmConstants.intakePosition));
    // operator.get(Button.Y).onTrue(new ExtendToPosition(arm, ArmConstants.shelfPosition));
    operator.get(Button.X).onTrue(new InstantCommand(() -> {
      intake.intake(IntakeConstants.kIntakeSpeed);
    System.out.println("X");
    },intake));

    operator.get(Button.B).onTrue(new InstantCommand(
      () -> {
      intake.intake(IntakeConstants.kOuttakeSpeed);
    },intake));

    operator.get(Button.Y).onTrue(new InstantCommand(() -> {
      intake.stop();
    },intake));
    
  }

  

  
}
