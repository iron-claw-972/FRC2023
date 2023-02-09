package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.ElevatorCalibrationAbsoluteEncoder;
import frc.robot.commands.ElevatorCalibrationLimitSwitch;
import frc.robot.commands.MoveToHeight;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Elevator;
import lib.controllers.GameController;
import lib.controllers.GameController.Axis;
import lib.controllers.GameController.Button;
import lib.controllers.GameController.DPad;

public class Operator {
  private static GameController operator = new GameController(Constants.oi.kOperatorJoy);

  public static void configureControls(Elevator elevator) {

    //Move to max height
    operator.get(operator.LEFT_TRIGGER_BUTTON).onTrue(new MoveToHeight(elevator, Constants.elevator.kElevatorTopHeightInches)); 

    //Move to min height
    operator.get(operator.RIGHT_TRIGGER_BUTTON).onTrue(new MoveToHeight(elevator, Constants.elevator.kElevatorBottomHeightInches)); 

    //Calibrate Elevator using limit switches
    operator.get(DPad.UP).onTrue(new ElevatorCalibrationLimitSwitch(elevator));
    
    //Calibrate elevator using absolute encodesr
    operator.get(DPad.DOWN).onTrue(new ElevatorCalibrationAbsoluteEncoder(elevator));
  
  }

  public static double getRawThrottleValue() {

    return operator.get(Axis.LEFT_Y);

  }
        
}
