package frc.robot.controls;

import frc.robot.commands.arm.ExtendToPosition;
import frc.robot.commands.elevator.MoveToHeight;
import frc.robot.commands.elevator.ResetEncoderAtBottom;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.OIConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.FourBarArm;
import lib.controllers.GameController;
import lib.controllers.GameController.Button;
import lib.controllers.GameController.DPad;
import lib.controllers.GameController.Axis;;

public class Operator {
  private static GameController operator = new GameController(OIConstants.kOperatorJoy);
  /**
   * Configures all of the operator controls.
   */
  public static void configureControls(Elevator elevator, FourBarArm arm) {
    //TODO: change keybinds so that the same one doesn't call two things. This may or may not be needed to be done. 
    //Move to max height
    operator.get(operator.LEFT_TRIGGER_BUTTON).onTrue(new MoveToHeight(elevator, ElevatorConstants.kMaxHeight)); 
    //Move to min height
    operator.get(operator.RIGHT_TRIGGER_BUTTON).onTrue(new MoveToHeight(elevator, ElevatorConstants.kBottomHeight)); 
    //Calibrate elevator using inbuilt motor encoders
    operator.get(DPad.DOWN).onTrue(new ResetEncoderAtBottom(elevator));
    //TODO: calibrate elevator using absolute encoders(probably will not work yet as of 2/15/2023);
    //move to bottom node height
    operator.get(Button.A).onTrue(new MoveToHeight(elevator, ElevatorConstants.kHeightBottomNode));
    //move to mid node height
    operator.get(Button.B).onTrue(new MoveToHeight(elevator, ElevatorConstants.kHeightMiddleNode));
    //move to top node height
    operator.get(Button.Y).onTrue(new MoveToHeight(elevator, ElevatorConstants.kHeightTopNode));
    operator.get(Button.Y).onTrue(new ExtendToPosition(arm, ArmConstants.topPosition));
    operator.get(Button.X).onTrue(new ExtendToPosition(arm, ArmConstants.middlePosition));
    operator.get(Button.A).onTrue(new ExtendToPosition(arm, ArmConstants.intakePosition));
    operator.get(Button.B).onTrue(new ExtendToPosition(arm, ArmConstants.shelfPosition));
  }

  public static double getRawThrottleValue() {
    return operator.get(Axis.LEFT_Y);
  }
}
