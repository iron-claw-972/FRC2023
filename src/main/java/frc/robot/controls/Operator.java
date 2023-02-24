package frc.robot.controls;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.constants.OIConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.commands.arm.ExtendToPosition;
import frc.robot.constants.ArmConstants;
import frc.robot.subsystems.FourBarArm;
import frc.robot.subsystems.Intake;
import frc.robot.util.Node;
import frc.robot.util.Vision;
import lib.controllers.GameController;
import lib.controllers.GameController.Axis;
import lib.controllers.GameController.Button;
import lib.controllers.GameController.DPad;


public class Operator {
  /// Selection values (grid, row, spot) for selecting where to score
  // For example, {1, 2, 3} means the left grid, middle row, and right spot in that grid
  public static int[] selectValues = {1, 1, 1};

  // Timer for clearing array. When this is equal to 0, selectValues is reset.
  public static double selectTime = 0;

  // How much time (in frames) before selection array is cleared
  public final static double selectTimeAmount=100 + Double.POSITIVE_INFINITY;
  
  // Where the robot will score
  public static Node selectedNode = new Node();

  private GameController m_operator;

  private Drivetrain m_drive;
  private FourBarArm m_arm;
  private Vision m_vision;
  private Intake m_intake;

  public Operator(Drivetrain drive, FourBarArm arm, Vision vision, Intake intake){
    m_drive=drive;
    m_arm=arm;
    m_vision=vision;
    m_intake=intake;
    m_operator = new GameController(OIConstants.kOperatorJoy);
    selectValue(0, 1);
  }

  public void configureControls() {

    // operator.get(Button.A).whenPressed(new DoNothing());

    // This will be added/fixed in a separate PR
    // m_operator.get(LEFT_STICK_LEFT).onTrue(new InstantCommand(()->selectValue(0, 1)));
    // m_operator.get(LEFT_STICK_UP).onTrue(new InstantCommand(()->selectValue(0, 2)));
    // m_operator.get(LEFT_STICK_RIGHT).onTrue(new InstantCommand(()->selectValue(0, 3)));

    // m_operator.get(RIGHT_STICK_LEFT).onTrue(new InstantCommand(()->selectValue(2, 1)));
    // m_operator.get(RIGHT_STICK_UP).onTrue(new InstantCommand(()->selectValue(2, 2)));
    // m_operator.get(RIGHT_STICK_RIGHT).onTrue(new InstantCommand(()->selectValue(2, 3)));

    // m_operator.get(Button.A).onTrue(new ParallelCommandGroup(new InstantCommand(()->selectValue(1, 1)), new ExtendToPosition(m_arm, ArmConstants.kLowPosition)));
    // m_operator.get(Button.X).onTrue(new ParallelCommandGroup(new InstantCommand(()->selectValue(1, 2)), new ExtendToPosition(m_arm, ArmConstants.kMiddlePosition)));
    // m_operator.get(Button.Y).onTrue(new ParallelCommandGroup(new InstantCommand(()->selectValue(1, 3)), new ExtendToPosition(m_arm, ArmConstants.kTopPosition)));
    
    // m_operator.get(Button.B).onTrue(new ExtendToPosition(m_arm, ArmConstants.kShelfPosition));
    // m_operator.get(Button.LB).onTrue(new ExtendToPosition(m_arm, ArmConstants.kInitialPosition));
  }

  public void selectValue(int index, int value){
    selectValues[index] = value;
    selectedNode = new Node(m_vision, DriverStation.getAlliance(), selectValues[1], selectValues[0]*3-3+selectValues[2]);
    System.out.println(selectedNode.scorePose);
}

private final BooleanSupplier LEFT_STICK_LEFT = () -> m_operator.get(Axis.LEFT_X) < -0.75,
LEFT_STICK_RIGHT = () -> m_operator.get(Axis.LEFT_X) > 0.75,
LEFT_STICK_UP = () -> m_operator.get(Axis.LEFT_Y) < -0.75,
LEFT_STICK_DOWN = () -> m_operator.get(Axis.LEFT_Y) > 0.75;
private final BooleanSupplier RIGHT_STICK_LEFT = () -> m_operator.get(Axis.RIGHT_X) < -0.75,
RIGHT_STICK_RIGHT = () -> m_operator.get(Axis.RIGHT_X) > 0.75,
RIGHT_STICK_UP = () -> m_operator.get(Axis.RIGHT_Y) < -0.75,
RIGHT_STICK_DOWN = () -> m_operator.get(Axis.RIGHT_Y) > 0.75;

  /**
   * This is not currently used because we are using other controls
   * 
   * Method to store DPad values and use them to set selectedNode
   * Down clears the array
   * Left is 1, up is 2, and right is 3 for selection
   * For example, up right left will select the center grid, top row, and left spot.
   * @param direction = Which DPad button is pressed
   */
  public void DPadPress(DPad direction) { 
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

        selectValue(0, selectValues[0]);
      }
    }

  }

}
