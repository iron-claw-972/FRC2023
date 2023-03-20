package frc.robot.controls;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.scoring.PositionIntake;
import frc.robot.commands.scoring.PositionIntake.Position;
import frc.robot.commands.scoring.elevator.CalibrateElevator;
import frc.robot.commands.scoring.intake.IntakeGamePiece;
import frc.robot.commands.scoring.intake.OuttakeGamePiece;
import frc.robot.commands.scoring.wrist.RotateWrist;
import frc.robot.constants.OIConstants;
import frc.robot.constants.WristConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.RollerIntake;
import frc.robot.subsystems.RollerIntake.IntakeMode;
import frc.robot.subsystems.Wrist;
import frc.robot.util.GamePieceType;
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
  
  /**
   * Configures the operator controls for the wrist, roller intake, elevator, and vision
   */
  public void configureControls(Wrist wrist, RollerIntake intake, Elevator elevator, Vision vision) {

    // calibrate elevator
    m_operator.get(Button.BACK).onTrue(new CalibrateElevator(elevator));

    // cancel all
    m_operator.get(DPad.UP).onTrue(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()));

    //top
    m_operator.get(Button.Y).onTrue(new PositionIntake(elevator, wrist, m_operator.RIGHT_TRIGGER_BUTTON, Position.TOP));
    //middle
    m_operator.get(Button.X).onTrue(new PositionIntake(elevator, wrist, m_operator.RIGHT_TRIGGER_BUTTON, Position.MIDDLE));
    //bottom
    m_operator.get(Button.A).onTrue(new PositionIntake(elevator, wrist, m_operator.RIGHT_TRIGGER_BUTTON, Position.BOTTOM));
    //shelf
    m_operator.get(Button.B).onTrue(new PositionIntake(elevator, wrist, () -> true, Position.SHELF).alongWith(new IntakeGamePiece(intake, () -> true, true)))
      .onFalse(new SequentialCommandGroup( 
        new InstantCommand(() -> intake.setMode(IntakeMode.DISABLED)),
        new InstantCommand(() -> intake.setHeldGamePiece(GamePieceType.CONE)),
        // for shelf, to not hit the shelf, move wrist slightly first
        new RotateWrist(wrist, WristConstants.kBottomNodeCubePos),
        new PositionIntake(elevator, wrist, m_operator.RIGHT_TRIGGER_BUTTON, Position.STOW)
      ));
    
    // stow
    m_operator.get(Button.RB).onTrue(new PositionIntake(elevator, wrist, m_operator.RIGHT_TRIGGER_BUTTON, Position.STOW));

    // intake
    m_operator.get(Button.LB).onTrue(
      new PositionIntake(elevator, wrist, m_operator.RIGHT_TRIGGER_BUTTON, Position.INTAKE).alongWith(new IntakeGamePiece(intake, m_operator.RIGHT_TRIGGER_BUTTON, true)))
      .onFalse(new SequentialCommandGroup(
        new InstantCommand(() -> intake.setMode(IntakeMode.DISABLED)),
        new PositionIntake(elevator, wrist, m_operator.RIGHT_TRIGGER_BUTTON, Position.STOW)
      ));

    // outtake
    m_operator.get(m_operator.LEFT_TRIGGER_BUTTON).onTrue(new OuttakeGamePiece(intake)).onFalse(new PositionIntake(elevator, wrist, m_operator.RIGHT_TRIGGER_BUTTON, Position.STOW).andThen(new InstantCommand(() -> intake.setMode(IntakeMode.DISABLED), intake)));

    // spin intake
    m_operator.get(DPad.DOWN).onTrue(new IntakeGamePiece(intake, m_operator.RIGHT_TRIGGER_BUTTON, true)).onFalse(new InstantCommand(() -> intake.setMode(IntakeMode.DISABLED), intake));
  
    // Selects which grid to score in
    // m_operator.get(m_operator.LEFT_STICK_LEFT).onTrue(new InstantCommand(() -> selectValue(NodePositionIndex.GRID, 2)));
    // m_operator.get(m_operator.LEFT_STICK_DOWN).onTrue(new InstantCommand(() -> selectValue(NodePositionIndex.GRID, 1)));
    // m_operator.get(m_operator.LEFT_STICK_RIGHT).onTrue(new InstantCommand(() -> selectValue(NodePositionIndex.GRID, 0)));
    // // Selects which column in the grid to score in
    // m_operator.get(m_operator.RIGHT_STICK_LEFT).onTrue(new InstantCommand(() -> selectValue(NodePositionIndex.COLUMN, 2)));
    // m_operator.get(m_operator.RIGHT_STICK_DOWN).onTrue(new InstantCommand(() -> selectValue(NodePositionIndex.COLUMN, 1)));
    // m_operator.get(m_operator.RIGHT_STICK_RIGHT).onTrue(new InstantCommand(() -> selectValue(NodePositionIndex.COLUMN, 0)));
    // Selects the row
    m_operator.get(Button.Y).onTrue(new InstantCommand(() -> selectValue(NodePositionIndex.ROW, 2)));
    m_operator.get(Button.X).onTrue(new InstantCommand(() -> selectValue(NodePositionIndex.ROW, 1)));
    m_operator.get(Button.A).onTrue(new InstantCommand(() -> selectValue(NodePositionIndex.ROW, 0)));
  }

  /**
   * Sets up shuffleboard
   * @param tab The tab on shuffleboard
   */
  public void setUpShuffleboard(ShuffleboardTab tab){
    tab.addStringArray("Selected node", ()->new String[]{
      "Alliance: "+getSelectedNode().alliance,
      "Type: "+getSelectedNode().type,
      "Row: "+getSelectedNode().row,
      "Column: "+getSelectedNode().column,
      String.format("Score pose: (%.2f, %.2f) at %d degrees", 
        getSelectedNode().scorePose.getX(), 
        getSelectedNode().scorePose.getY(), 
        getSelectedNode().scorePose.getRotation().getDegrees()
      )
    });
  }

  /**
   * This function puts a number in selectValues and uses those values to select a node.
   * THESE VALUES ARE RELATIVE TO THE BLUE ALLIANCE, AND WILL BE SWITCHED FOR RED AUTOMATICALLY
   * @param index Which item in the array to change
   * @param value What value to set it to, between 0 and 2 (0 is lower row (hybrid node), or for column/grid the closest one to field boundry)
   */
  public void selectValue(NodePositionIndex index, int value) {
    if (index!=NodePositionIndex.ROW && DriverStation.getAlliance() == Alliance.Red) {value = 2-value;}
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
