package frc.robot.controls;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.scoring.Dunk;
import frc.robot.commands.scoring.PositionIntake;
import frc.robot.commands.scoring.PositionIntake.Position;
import frc.robot.commands.scoring.Stow;
import frc.robot.commands.scoring.arm.ExtendArm;
import frc.robot.commands.scoring.elevator.CalibrateElevator;
import frc.robot.commands.scoring.elevator.MoveElevator;
import frc.robot.commands.scoring.intake.IntakeGamePiece;
import frc.robot.commands.scoring.intake.Outtake;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.OIConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.FourBarArm;
import frc.robot.subsystems.Intake;
import frc.robot.util.Node;
import frc.robot.util.Vision;
import lib.controllers.GameController;
import lib.controllers.GameController.Button;
import lib.controllers.GameController.DPad;

public class Operator {

  private GameController m_operator = new GameController(OIConstants.kOperatorJoy);

  // Values for selecting a node
  // Grid, row, and column in the grid
  private int[] selectValues = {1, 1, 1};
  private Node m_selectedNode;
  private Vision m_vision;
  
  /**
   * Configures the operator controls for the deploying Bar.
   */
  public void configureControls(FourBarArm arm, Intake intake, Elevator elevator, Vision vision) {

    m_vision = vision; //should be in constructor

    m_operator.get(Button.BACK).onTrue(new CalibrateElevator(elevator));


    m_operator.get(DPad.UP).onTrue(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()));

    //top
    m_operator.get(Button.Y).onTrue(new PositionIntake(elevator, arm, () -> true, Position.TOP));
    //middle
    m_operator.get(Button.X).onTrue(new PositionIntake(elevator, arm, () -> true, Position.MIDDLE));
    //bottom
    m_operator.get(Button.A).onTrue(new PositionIntake(elevator, arm, () -> true, Position.BOTTOM));
    //shelf
    m_operator.get(Button.B).onTrue(new PositionIntake(elevator, arm, () -> true, Position.SHELF).alongWith(new IntakeGamePiece(intake)))
      .onFalse(new SequentialCommandGroup( 
        new InstantCommand(() -> intake.stopIntake()),
        new ExtendArm(arm, 0.8),
        new MoveElevator(elevator, ElevatorConstants.kStowHeight),
        new Stow(intake, elevator, arm)
      ));
    
    //stow
    m_operator.get(Button.RB).onTrue(new Stow(intake, elevator, arm));

    //intake
    m_operator.get(Button.LB).onTrue(
      new PositionIntake(elevator, arm, () -> true, Position.INTAKE).alongWith(new IntakeGamePiece(intake)))
      .onFalse(new Stow(intake, elevator, arm));

    //dunk
    m_operator.get(m_operator.RIGHT_TRIGGER_BUTTON).onTrue(new Dunk(arm, intake)).onFalse(new Stow(intake, elevator, arm));

    //outtake
    m_operator.get(m_operator.LEFT_TRIGGER_BUTTON).onTrue(new Outtake(intake, false)).onFalse(new Stow(intake, elevator, arm));
  
  
    // Selects which grid to score in
    m_operator.get(m_operator.LEFT_STICK_LEFT).onTrue(new InstantCommand(() -> selectValue(NodePositionIndex.GRID, 2)));
    m_operator.get(m_operator.LEFT_STICK_DOWN).onTrue(new InstantCommand(() -> selectValue(NodePositionIndex.GRID, 1)));
    m_operator.get(m_operator.LEFT_STICK_RIGHT).onTrue(new InstantCommand(() -> selectValue(NodePositionIndex.GRID, 0)));
    // Selects which column in the grid to score in
    m_operator.get(m_operator.RIGHT_STICK_LEFT).onTrue(new InstantCommand(() -> selectValue(NodePositionIndex.COLUMN, 2)));
    m_operator.get(m_operator.RIGHT_STICK_DOWN).onTrue(new InstantCommand(() -> selectValue(NodePositionIndex.COLUMN, 1)));
    m_operator.get(m_operator.RIGHT_STICK_RIGHT).onTrue(new InstantCommand(() -> selectValue(NodePositionIndex.COLUMN, 0)));
    // Selects the row
    m_operator.get(DPad.LEFT).onTrue(new InstantCommand(() -> selectValue(NodePositionIndex.ROW, 2)));
    m_operator.get(DPad.DOWN).onTrue(new InstantCommand(() -> selectValue(NodePositionIndex.ROW, 1)));
    m_operator.get(DPad.RIGHT).onTrue(new InstantCommand(() -> selectValue(NodePositionIndex.ROW, 0)));
  }

  /**
   * This function puts a number in selectValues and uses those values to select a node.
   * THESE VALUES ARE RELATIVE TO THE BLUE ALLIANCE, AND WILL BE SWITCHED FOR RED AUTOMATICALLY
   * @param index Which item in the array to change
   * @param value What value to set it to, between 0 and 2 (0 is lower row (hybrid node), or for column/grid the closest one to field boundry)
   */
  public void selectValue(NodePositionIndex index, int value) {
    if (value == 0 && DriverStation.getAlliance() == Alliance.Red) value = 2;
    if (value == 2 && DriverStation.getAlliance() == Alliance.Red) value = 0;
    selectValues[index.id] = value;
    m_selectedNode = new Node(DriverStation.getAlliance(), selectValues[NodePositionIndex.ROW.id]+1, (selectValues[NodePositionIndex.GRID.id]*3)+selectValues[NodePositionIndex.COLUMN.id]+1);
  }

  public Node getSelectedNode() {
    return m_selectedNode;
  }

  private enum NodePositionIndex {
    GRID(0), ROW(1), COLUMN(2);
    int id;
    private NodePositionIndex(int id) {
      this.id = id;
    }
  }
}
