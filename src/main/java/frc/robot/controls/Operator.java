package frc.robot.controls;

import frc.robot.commands.ElevatorCalibrationAbsoluteEncoder;
import frc.robot.commands.MoveToHeight;
import frc.robot.commands.arm.ExtendToPosition;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.OIConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.FourBarArm;
import lib.controllers.GameController;
import lib.controllers.GameController.Button;
import lib.controllers.GameController.DPad;

public class Operator {
  private static GameController operator = new GameController(OIConstants.kOperatorJoy);

  public static void configureControls(Elevator elevator) {

    //Move to max height
    operator.get(operator.LEFT_TRIGGER_BUTTON).onTrue(new MoveToHeight(elevator, ElevatorConstants.kMaxHeight)); 

    //Move to min height
    operator.get(operator.RIGHT_TRIGGER_BUTTON).onTrue(new MoveToHeight(elevator, ElevatorConstants.kBottomHeightMeters)); 
    
    //Calibrate elevator using absolute encoders
    operator.get(DPad.DOWN).onTrue(new ElevatorCalibrationAbsoluteEncoder(elevator));

    //move to bottom node height
    operator.get(Button.A).onTrue(new MoveToHeight(elevator, ElevatorConstants.kHeightBottomNodeMeters));

    //move to mid node height
    operator.get(Button.B).onTrue(new MoveToHeight(elevator, ElevatorConstants.kHeightMiddleNodeMeters));

    //move to top node height
    operator.get(Button.Y).onTrue(new MoveToHeight(elevator, ElevatorConstants.kHeightTopNodeMeters));
  }

  public static double getRawThrottleValue() {
    return operator.get(Axis.LEFT_Y);
  /**
   * Configures all of the operator controls.
   */
  public static void configureControls(FourBarArm arm) {
    operator.get(Button.Y).onTrue(new ExtendToPosition(arm, ArmConstants.topPosition));
    operator.get(Button.X).onTrue(new ExtendToPosition(arm, ArmConstants.middlePosiiton));
    operator.get(Button.A).onTrue(new ExtendToPosition(arm, ArmConstants.intakePosition));
    operator.get(Button.Y).onTrue(new ExtendToPosition(arm, ArmConstants.shelfPosition));
  }
}
