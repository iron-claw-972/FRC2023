package frc.robot.controls;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DoNothing;
import frc.robot.commands.scoring.PositionIntake;
import frc.robot.commands.scoring.Stow;
import frc.robot.commands.scoring.PositionIntake.Position;
import frc.robot.commands.scoring.elevator.CalibrateElevator;
import frc.robot.commands.scoring.intake.HoldCone;
import frc.robot.commands.scoring.intake.IntakeGamePiece;
import frc.robot.commands.scoring.intake.OuttakeGamePiece;
import frc.robot.commands.scoring.wrist.RotateWrist;
import frc.robot.constants.OIConstants;
import frc.robot.constants.WristConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeMode;
import frc.robot.subsystems.Wrist;
import frc.robot.util.Blinkin;
import frc.robot.util.GamePieceType;
import frc.robot.util.Node;
import frc.robot.util.Vision;
import frc.robot.util.Blinkin.Colors;
import lib.controllers.GameController;
import lib.controllers.GameController.Button;
import lib.controllers.GameController.DPad;

public class Operator {

  private GameController m_operator = new GameController(OIConstants.kOperatorJoy);

  // Values for selecting a node
  // the row (hybrid, middle, top) for scoring in
  private int row = 3;
  // the column, 1 - 9. 1 is closest to the field boundary. 9 is closest to loading zone
  private int column = 0;
  // the currently selected node.
  private Node m_selectedNode = new Node();
  
  /**
   * Configures the operator controls for the wrist, roller intake, elevator, and vision
   */
  public void configureControls(Wrist wrist, Intake intake, Elevator elevator, Vision vision) {

    // calibrate elevator
    m_operator.get(Button.BACK).onTrue(new CalibrateElevator(elevator));

    // cancel all
    m_operator.get(DPad.UP).onTrue(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()));

    //top
    m_operator.get(Button.Y).onTrue(new PositionIntake(elevator, wrist, m_operator.RIGHT_TRIGGER_BUTTON, Position.TOP)
      .alongWith(new ConditionalCommand(new HoldCone(intake), new DoNothing(), m_operator.RIGHT_TRIGGER_BUTTON)));
    //middle
    m_operator.get(Button.X).onTrue(new PositionIntake(elevator, wrist, m_operator.RIGHT_TRIGGER_BUTTON, Position.MIDDLE)
      .alongWith(new ConditionalCommand(new HoldCone(intake), new DoNothing(), m_operator.RIGHT_TRIGGER_BUTTON)));
    //bottom
    m_operator.get(Button.A).onTrue(new PositionIntake(elevator, wrist, m_operator.RIGHT_TRIGGER_BUTTON, Position.BOTTOM)
      .alongWith(new ConditionalCommand(new HoldCone(intake), new DoNothing(), m_operator.RIGHT_TRIGGER_BUTTON)));
    //shelf
    m_operator.get(Button.B).onTrue(new PositionIntake(elevator, wrist, () -> true, Position.SHELF).alongWith(new IntakeGamePiece(intake, () -> true, true)))
      .onFalse(new SequentialCommandGroup( 
        new InstantCommand(() -> intake.setMode(IntakeMode.DISABLED)),
        new InstantCommand(() -> intake.setHeldGamePiece(GamePieceType.CONE)),
        // for shelf, to not hit the shelf, move wrist slightly first
        new RotateWrist(wrist, WristConstants.kBottomNodeCubePos),
        new Stow(elevator, wrist)
      ));
    
    // stow
    m_operator.get(Button.RB).onTrue(new Stow(elevator, wrist));

    // intake
    m_operator.get(Button.LB).onTrue(
      new PositionIntake(elevator, wrist, m_operator.RIGHT_TRIGGER_BUTTON, Position.INTAKE).alongWith(new IntakeGamePiece(intake, m_operator.RIGHT_TRIGGER_BUTTON, true)))
      .onFalse(new SequentialCommandGroup(
        new InstantCommand(() -> intake.setMode(IntakeMode.DISABLED)),
        new Stow(elevator, wrist)
      ));

    // LED controls
    m_operator.get(Button.LEFT_JOY).onTrue(new ConditionalCommand(
      new InstantCommand(() -> Blinkin.blinkColor(Colors.YELLOW)),
      new InstantCommand(() -> Blinkin.blinkColor(Colors.VIOLET)), 
      m_operator.RIGHT_TRIGGER_BUTTON));

    // outtake
    m_operator.get(m_operator.LEFT_TRIGGER_BUTTON).whileTrue(new OuttakeGamePiece(intake)).onFalse(new Stow(elevator, wrist));

    // spin intake
    m_operator.get(DPad.DOWN).onTrue(new IntakeGamePiece(intake, m_operator.RIGHT_TRIGGER_BUTTON, true)).onFalse(new InstantCommand(() -> intake.setMode(IntakeMode.DISABLED), intake));
  
    // Selects which column to score in
    m_operator.get(m_operator.LEFT_STICK_LEFT).onTrue(new InstantCommand(() -> selectColumn(9, true)));
    m_operator.get(m_operator.LEFT_STICK_UP).onTrue(new InstantCommand(() -> selectColumn(8, true)));
    m_operator.get(m_operator.LEFT_STICK_RIGHT).onTrue(new InstantCommand(() -> selectColumn(7, true)));
    
    m_operator.get(DPad.LEFT).onTrue(new InstantCommand(() -> selectColumn(6, true)));
    m_operator.get(DPad.UP).onTrue(new InstantCommand(() -> selectColumn(5, true)));
    m_operator.get(DPad.RIGHT).onTrue(new InstantCommand(() -> selectColumn(4, true)));

    m_operator.get(m_operator.RIGHT_STICK_LEFT).onTrue(new InstantCommand(() -> selectColumn(3, true)));
    m_operator.get(m_operator.RIGHT_STICK_UP).onTrue(new InstantCommand(() -> selectColumn(2, true)));
    m_operator.get(m_operator.RIGHT_STICK_RIGHT).onTrue(new InstantCommand(() -> selectColumn(1, true)));

    // Selects the row
    m_operator.get(Button.Y).onTrue(new InstantCommand(() -> selectRow(3)));
    m_operator.get(Button.X).onTrue(new InstantCommand(() -> selectRow(2)));
    m_operator.get(Button.A).onTrue(new InstantCommand(() -> selectRow(1)));
  }

  /**
   * Sets up shuffleboard
   * @param tab The tab on shuffleboard
   */
  public void setUpShuffleboard(ShuffleboardTab tab) {
    tab.addStringArray("Selected node", () -> new String[] {
      "Alliance: " + getSelectedNode().alliance,
      "Type: " + getSelectedNode().type,
      "Row: " + getSelectedNode().row,
      "Column: " + getSelectedNode().column,
      String.format("Score pose: (%.2f, %.2f) at %f degrees", 
        getSelectedNode().scorePose.getX(), 
        getSelectedNode().scorePose.getY(), 
        getSelectedNode().scorePose.getRotation().getDegrees()
      )
    });
    tab.addDouble("Column Alignment", () -> getSelectedNode().column);
  }

  /**
   * Selects which row (hybrid, middle, top) to score in
   * @param value What value to set it to, between 1 and 3 (1 is lower row (hybrid node))
   */
  private void selectRow(int value) {
    row = value;
    m_selectedNode = new Node(DriverStation.getAlliance(), row, column);
  }

  /**
   * Selects which column (1-9) to score in. 1 is closest to field boundary, 9 is closest to loading zone
   * @param value What value to set it to, between 1 and 9
   * @param allianceRelative if true, will swap the grids if on red alliance.
   */
  private void selectColumn(int value, boolean allianceRelative) {
    if (allianceRelative && DriverStation.getAlliance() == Alliance.Red) {value = 10 - value;}
    column = value;
    m_selectedNode = new Node(DriverStation.getAlliance(), row, column);
  }

  public Node getSelectedNode() {
    return m_selectedNode;
  }
}
