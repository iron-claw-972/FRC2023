package frc.robot.controls;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.commands.arm.ExtendToPosition;
import frc.robot.commands.DoNothing;
import frc.robot.constants.ArmConstants;
import frc.robot.commands.deployingbar.RotateDeployingBar;
import frc.robot.constants.DeployingBarConstants;
import frc.robot.constants.OIConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.FourBarArm;
import frc.robot.subsystems.DeployingBar;
import frc.robot.subsystems.Intake;
import frc.robot.util.NodeLegecy;
import frc.robot.util.Vision;
import lib.controllers.GameController;
import lib.controllers.GameController.Axis;
import lib.controllers.GameController.Button;
import lib.controllers.GameController.DPad;

public class Operator {

  private GameController m_operator = new GameController(OIConstants.kOperatorJoy);

  private Drivetrain m_drive;
  private FourBarArm m_arm;
  private Vision m_vision;
  private Intake m_intake;
  private DeployingBar m_deployingBar;

  // Values for selecting a node
  // Grid, row, and column in the grid
  private int[] selectValues = {1, 1, 1};

  // The selected node where the robot will score
  public static NodeLegecy selectedNode = new NodeLegecy();

  public Operator(Drivetrain drive, FourBarArm arm, Vision vision, Intake intake, DeployingBar deployingBar){
    m_drive = drive;
    m_arm = arm;
    m_vision = vision;
    m_intake = intake;
    m_deployingBar = deployingBar;
    m_operator = new GameController(OIConstants.kOperatorJoy);
    selectValue(0, 1);
  }
  
  /**
   * Configures the operator controls for the deploying Bar.
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

    // Makes the arm and elevator go to different heights
    m_operator.get(Button.A).onTrue(new ParallelCommandGroup(
      new InstantCommand(() -> selectValue(1, 1)),
      m_arm==null?new DoNothing():new ExtendToPosition(m_arm, ArmConstants.kLowPosition)
    ));
    m_operator.get(Button.X).onTrue(new ParallelCommandGroup(
      new InstantCommand(() -> selectValue(1, 2)),
      m_arm==null?new DoNothing():new ExtendToPosition(m_arm, ArmConstants.kMiddlePosition)
    ));
    m_operator.get(Button.Y).onTrue(new ParallelCommandGroup(
      new InstantCommand(() -> selectValue(1, 3)),
      m_arm==null?new DoNothing():new ExtendToPosition(m_arm, ArmConstants.kTopPosition)
    ));
          
    if(m_arm!=null && m_intake!=null){
      // Puts the arm and elevator in the initial position inside the robot
      m_operator.get(Button.LB).onTrue(new ExtendToPosition(m_arm, ArmConstants.kInitialPosition));
      
      // Intakes from shelf or ground
      m_operator.get(m_operator.RIGHT_TRIGGER_BUTTON).onTrue(new ParallelCommandGroup(
        new InstantCommand(() -> m_intake.intake(1)),
        new ExtendToPosition(m_arm, ArmConstants.kShelfPosition)
      ));
      m_operator.get(m_operator.RIGHT_TRIGGER_BUTTON).onFalse(new ParallelCommandGroup(
        new InstantCommand(() -> m_intake.stop()), 
        new ExtendToPosition(m_arm, ArmConstants.kInitialPosition)
      ));
      m_operator.get(Button.RB).onTrue(new ParallelCommandGroup(
        new InstantCommand(() -> m_intake.intake(1)), 
        new ExtendToPosition(m_arm, ArmConstants.kIntakePosition)
      ));
      m_operator.get(Button.RB).onFalse(new ParallelCommandGroup(
        new InstantCommand(() -> m_intake.stop()), 
        new ExtendToPosition(m_arm, ArmConstants.kInitialPosition)
      ));

      // Outtake
      m_operator.get(m_operator.LEFT_TRIGGER_BUTTON).onTrue(new InstantCommand(() -> m_intake.intake(-1)));
      m_operator.get(m_operator.LEFT_TRIGGER_BUTTON).onFalse(new InstantCommand(() -> m_intake.stop()));
      
      // Extends the bar
      m_operator.get(Button.B).onTrue(new RotateDeployingBar(m_deployingBar, DeployingBarConstants.kDeployedRotation));
      m_operator.get(Button.B).onFalse(new RotateDeployingBar(m_deployingBar, DeployingBarConstants.kStowRotation));
    }

    // Prints that operator controls are working
    m_operator.get(DPad.UP).onTrue(new PrintCommand("Operator controls work"));
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

  private final BooleanSupplier 
    LEFT_STICK_LEFT = () -> m_operator.get(Axis.LEFT_X) < -0.75,
    LEFT_STICK_RIGHT = () -> m_operator.get(Axis.LEFT_X) > 0.75,
    LEFT_STICK_UP = () -> m_operator.get(Axis.LEFT_Y) < -0.75;
  private final BooleanSupplier 
    RIGHT_STICK_LEFT = () -> m_operator.get(Axis.RIGHT_X) < -0.75,
    RIGHT_STICK_RIGHT = () -> m_operator.get(Axis.RIGHT_X) > 0.75,
    RIGHT_STICK_UP = () -> m_operator.get(Axis.RIGHT_Y) < -0.75;
}
