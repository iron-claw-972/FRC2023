package frc.robot.controls;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.arm.ExtendToPosition;
import frc.robot.constants.ArmConstants;
import frc.robot.commands.deployingbar.RotateDeployingBar;
import frc.robot.commands.elevator.MoveToExtension;
import frc.robot.constants.DeployingBarConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.OIConstants;
import frc.robot.subsystems.FourBarArm;
import frc.robot.subsystems.DeployingBar;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.util.NodeLegecy;
import frc.robot.util.Vision;
import lib.controllers.GameController;
import lib.controllers.GameController.Axis;
import lib.controllers.GameController.Button;

public class Operator {

  private GameController m_operator = new GameController(OIConstants.kOperatorJoy);

  private Vision m_vision;

  // Values for selecting a node
  // Grid, row, and column in the grid
  private int[] selectValues = {1, 1, 1};

  // The selected node where the robot will score
  private static NodeLegecy selectedNode = new NodeLegecy();

  private final BooleanSupplier 
    LEFT_STICK_LEFT = () -> m_operator.get(Axis.LEFT_X) < -0.75,
    LEFT_STICK_RIGHT = () -> m_operator.get(Axis.LEFT_X) > 0.75,
    LEFT_STICK_UP = () -> m_operator.get(Axis.LEFT_Y) < -0.75;
  private final BooleanSupplier 
    RIGHT_STICK_LEFT = () -> m_operator.get(Axis.RIGHT_X) < -0.75,
    RIGHT_STICK_RIGHT = () -> m_operator.get(Axis.RIGHT_X) > 0.75,
    RIGHT_STICK_UP = () -> m_operator.get(Axis.RIGHT_Y) < -0.75;

  public Operator(Vision vision){
    m_vision = vision;
    m_operator = new GameController(OIConstants.kOperatorJoy);
    selectValue(0, 1);
  }
  
  /**
   * Configures the operator controls for column selection.
   */
  public void configureControls() {
    // Selects which grid to score in
    m_operator.get(LEFT_STICK_LEFT).onTrue(new InstantCommand(() -> selectValue(0, 1)));
    m_operator.get(LEFT_STICK_UP).onTrue(new InstantCommand(() -> selectValue(0, 2)));
    m_operator.get(LEFT_STICK_RIGHT).onTrue(new InstantCommand(() -> selectValue(0, 3)));
    // Selects which spot in the grid to score in
    m_operator.get(RIGHT_STICK_LEFT).onTrue(new InstantCommand(() -> selectValue(2, 1)));
    m_operator.get(RIGHT_STICK_UP).onTrue(new InstantCommand(() -> selectValue(2, 2)));
    m_operator.get(RIGHT_STICK_RIGHT).onTrue(new InstantCommand(() -> selectValue(2, 3)));
  }

  /**
   * Configures arm controls
   * @param arm The arm
   */
  public void configureControls(FourBarArm arm){
    // Makes the arm go to different heights
    m_operator.get(Button.A).onTrue(new ParallelCommandGroup(
      new InstantCommand(() -> selectValue(1, 1)),
      new ExtendToPosition(arm, ArmConstants.kLowPosition)
    ));
    m_operator.get(Button.X).onTrue(new ParallelCommandGroup(
      new InstantCommand(() -> selectValue(1, 2)),
      new ExtendToPosition(arm, ArmConstants.kMiddlePosition)
    ));
    m_operator.get(Button.Y).onTrue(new ParallelCommandGroup(
      new InstantCommand(() -> selectValue(1, 3)),
      new ExtendToPosition(arm, ArmConstants.kTopPosition)
    ));

    // Puts the arm in the initial position inside the robot
    m_operator.get(Button.LB).onTrue(new ExtendToPosition(arm, ArmConstants.kInitialPosition));
      
    // Moves arm to position to intake from shelf or ground
    m_operator.get(m_operator.RIGHT_TRIGGER_BUTTON).onTrue(new ExtendToPosition(arm, ArmConstants.kShelfPosition));
    m_operator.get(m_operator.RIGHT_TRIGGER_BUTTON).onFalse(new ExtendToPosition(arm, ArmConstants.kInitialPosition));
    m_operator.get(Button.RB).onTrue(new ExtendToPosition(arm, ArmConstants.kIntakePosition));
    m_operator.get(Button.RB).onFalse(new ExtendToPosition(arm, ArmConstants.kInitialPosition));
  }

  /**
   * Configures elevator controls
   * @param elevator The elevator
   */
  public void configureControls(Elevator elevator){
    // Makes the elevator go to different heights
    m_operator.get(Button.A).onTrue(new MoveToExtension(elevator, ElevatorConstants.kHybridNodeOuttakeExtension));
    m_operator.get(Button.X).onTrue(new MoveToExtension(elevator, 
      selectValues[2]==2?ElevatorConstants.kMiddleNodeHeightCube:ElevatorConstants.kMiddleNodeHeightCone
    ));
    m_operator.get(Button.Y).onTrue(new MoveToExtension(elevator, 
      selectValues[2]==2?ElevatorConstants.kTopNodeHeightCube:ElevatorConstants.kTopNodeHeightCone
    ));


    // Puts the elevator in the initial position
    m_operator.get(Button.LB).onTrue(new MoveToExtension(elevator, ElevatorConstants.kMinExtension));
      
    // Moves elevator to position to intake from shelf or ground
    m_operator.get(m_operator.RIGHT_TRIGGER_BUTTON).onTrue(new ExtendToPosition(arm, ArmConstants.kShelfPosition));
    m_operator.get(m_operator.RIGHT_TRIGGER_BUTTON).onFalse(new ExtendToPosition(arm, ArmConstants.kInitialPosition));
    m_operator.get(Button.RB).onTrue(new ExtendToPosition(arm, ArmConstants.kIntakePosition));
    m_operator.get(Button.RB).onFalse(new ExtendToPosition(arm, ArmConstants.kInitialPosition));
  }

  /**
   * Configures intkae controls
   * @param intake The intake
   */
  public void configureControls(Intake intake){
    // Intake
    m_operator.get(m_operator.RIGHT_TRIGGER_BUTTON).onTrue(new InstantCommand(()->intake.intake(1)));
    m_operator.get(Button.RB).onTrue(new InstantCommand(()->intake.intake(1)));
    m_operator.get(m_operator.RIGHT_TRIGGER_BUTTON).onTrue(new InstantCommand(()->intake.stop()));
    m_operator.get(Button.RB).onTrue(new InstantCommand(()->intake.stop()));

    // Outtake
    m_operator.get(m_operator.LEFT_TRIGGER_BUTTON).onTrue(new InstantCommand(() -> intake.intake(-1)));
    m_operator.get(m_operator.LEFT_TRIGGER_BUTTON).onFalse(new InstantCommand(() -> intake.stop()));
  }

  /**
   * Configures deploying bar controls
   * @param deployingBar The deploying bar
   */
  public void configureControls(DeployingBar deployingBar){
    // Extends the bar
    m_operator.get(Button.B).onTrue(new RotateDeployingBar(deployingBar, DeployingBarConstants.kDeployedRotation));
    m_operator.get(Button.B).onFalse(new RotateDeployingBar(deployingBar, DeployingBarConstants.kStowRotation));
  }

  public static NodeLegecy getSelectedNode(){
    return selectedNode;
  }

  /**
   * This function puts a number in selectValues and uses those values to select a node
   * @param index Which item in the array to change (1 = grid, 2 = row, 3 = column in grid)
   * @param value What value to set it to, between 1 and 3 (1 is left or bottom)
   */
  public void selectValue(int index, int value){
    selectValues[index] = value;
    selectedNode = new NodeLegecy(m_vision, DriverStation.getAlliance(), selectValues[1], selectValues[0]*3-3+selectValues[2]);
  }
}
