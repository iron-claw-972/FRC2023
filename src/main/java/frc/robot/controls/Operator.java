package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ElevatorCalibrationAbsoluteEncoder;
import frc.robot.commands.MoveToHeight;
import frc.robot.commands.ResetEncoderAtBottom;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.OIConstants;
import frc.robot.subsystems.Elevator;
import lib.controllers.GameController;
import lib.controllers.GameController.Axis;
import lib.controllers.GameController.DPad;

public class Operator {
  private static GameController operator = new GameController(OIConstants.kOperatorJoy);

  public static void configureControls(Elevator elevator) {

    //Move to max height
    operator.get(operator.LEFT_TRIGGER_BUTTON).onTrue(new MoveToHeight(elevator, ElevatorConstants.kMaxHeight)); 

    //Move to min height
    operator.get(operator.RIGHT_TRIGGER_BUTTON).onTrue(new MoveToHeight(elevator, ElevatorConstants.kBottomHeightMeters)); 
    
    //Calibrate elevator using absolute encodesr
    operator.get(DPad.DOWN).onTrue(new ElevatorCalibrationAbsoluteEncoder(elevator));
  
  }

  public static double getRawThrottleValue() {

    return operator.get(Axis.LEFT_Y);

  }
        
}
