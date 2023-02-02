package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.MoveToBottomNode;
import frc.robot.commands.MoveToMaxHeight;
import frc.robot.commands.MoveToMinHeight;
import frc.robot.constants.Constants;
import frc.robot.subsystems.AngledElevator;
import lib.controllers.GameController;
import lib.controllers.GameController.Axis;
import lib.controllers.GameController.Button;
import lib.controllers.GameController.DPad;

public class Operator {
  private static GameController operator = new GameController(Constants.oi.kOperatorJoy);
  
  public static void configureControls() {

    operator.get(DPad.UP).onTrue();
    operator.get(DPad.DOWN).onTrue();
    operator.get(Button.A).onTrue(); 
    operator.get(Button.B).onTrue(); 
    operator.get(Button.Y).onTrue();

  }

  public static double getRawThrottleValue() {

    return operator.get(Axis.LEFT_Y);

  }
        
}
