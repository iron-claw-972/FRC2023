package frc.robot.controls;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.DoNothing;
import frc.robot.constants.Constants;
import frc.robot.constants.OIConstants;
import frc.robot.subsystems.Drivetrain;
import lib.controllers.GameController;
import lib.controllers.GameController.Button;
import lib.controllers.GameController.DPad;

public class Operator {
  /// Selection values (grid, row, spot) for selecting where to score
  // For example, {1, 2, 3} means the left grid, middle row, and right spot in that grid
  public static int[] selectValues = {0, 0, 0};

  // Timer for clearing array. When this is equal to 0, selectValues is reset.
  public static double selectTime = 0;

  // How much time (in frames) before selection array is cleared
  public final static double selectTimeAmount=100;

  private static GameController operator = new GameController(OIConstants.kOperatorJoy);

  public static void configureControls(Drivetrain drive) {
    // operator.get(Button.A).whenPressed(new DoNothing());
    operator.get(DPad.LEFT).onTrue(new InstantCommand(()->DPadPress(DPad.LEFT)));
    operator.get(DPad.UP).onTrue(new InstantCommand(()->DPadPress(DPad.UP)));
    operator.get(DPad.RIGHT).onTrue(new InstantCommand(()->DPadPress(DPad.RIGHT)));
    operator.get(DPad.DOWN).onTrue(new InstantCommand(()->DPadPress(DPad.DOWN)));
  }

    /**
   * Method to store DPad values and use them to set selectedNode
   * Down clears the array
   * Left is 1, up is 2, and right is 3 for selection
   * For example, up right left will select the center grid, top row, and left spot.
   * @param direction = Which DPad button is pressed
   */
  public static void DPadPress(DPad direction) { 
    if (direction==DPad.DOWN) {
      selectTime=1;
    } else {
      selectTime = selectTimeAmount;
      int pressValue = direction == DPad.LEFT?1:direction==DPad.UP?2:3;

      if (selectValues[0]==0){
        selectValues[0]=pressValue;
      } else if (selectValues[1]==0) {
        selectValues[1]=pressValue;
      } else {
        selectValues[2] = pressValue;
        selectTime = 1;

        if (RobotContainer.alliance == Alliance.Blue) {
          RobotContainer.selectedNode = RobotContainer.blueNodes[selectValues[1]][selectValues[0]*3-3+selectValues[2]];
        }else{
          RobotContainer.selectedNode = RobotContainer.redNodes[selectValues[1]][selectValues[0]*3-3+selectValues[2]];
        }
      }
    }
  }
}
