package frc.robot.controls;

import frc.robot.constants.OIConstants;
import frc.robot.subsystems.FourBarArm;
import frc.robot.subsystems.Intake;
import frc.robot.util.Node;
import frc.robot.util.Vision;
import lib.controllers.GameController;

public class Operator {

  private GameController m_operator = new GameController(OIConstants.kOperatorJoy);

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
    // Selects which grid to score in
    m_operator.get(LEFT_STICK_LEFT).onTrue(new InstantCommand(()->selectValue(0, 1)));
    m_operator.get(LEFT_STICK_UP).onTrue(new InstantCommand(()->selectValue(0, 2)));
    m_operator.get(LEFT_STICK_RIGHT).onTrue(new InstantCommand(()->selectValue(0, 3)));
    // Selects which spot in the grid to score in
    m_operator.get(RIGHT_STICK_LEFT).onTrue(new InstantCommand(()->selectValue(2, 1)));
    m_operator.get(RIGHT_STICK_UP).onTrue(new InstantCommand(()->selectValue(2, 2)));
    m_operator.get(RIGHT_STICK_RIGHT).onTrue(new InstantCommand(()->selectValue(2, 3)));

    // Makes the arm and elevator go to different heights
    m_operator.get(Button.A).onTrue(new ParallelCommandGroup(new InstantCommand(()->selectValue(1, 1)), new ExtendToPosition(m_arm, ArmConstants.klowPosition)));
    m_operator.get(Button.X).onTrue(new ParallelCommandGroup(new InstantCommand(()->selectValue(1, 2)), new ExtendToPosition(m_arm, ArmConstants.kmiddlePosition)));
    m_operator.get(Button.Y).onTrue(new ParallelCommandGroup(new InstantCommand(()->selectValue(1, 3)), new ExtendToPosition(m_arm, ArmConstants.ktopPosition)));
    
    // Puts the arm and elevator in the initial position inside the robot
    m_operator.get(Button.LB).onTrue(new ExtendToPosition(m_arm, ArmConstants.kInitialPosition));
    
    // Intakes from shelf or ground
    m_operator.get(m_operator.RIGHT_TRIGGER_BUTTON).onTrue(new ParallelCommandGroup(new InstantCommand(()->m_intake.intake(1)), new ExtendToPosition(m_arm, ArmConstants.kShelfPosition)));
    m_operator.get(m_operator.RIGHT_TRIGGER_BUTTON).onFalse(new ParallelCommandGroup(new InstantCommand(()->m_intake.stop()), new ExtendToPosition(m_arm, ArmConstants.kInitialPosition)));
    m_operator.get(Button.RB).onTrue(new ParallelCommandGroup(new InstantCommand(()->m_intake.intake(1)), new ExtendToPosition(m_arm, ArmConstants.kIntakePosition)));
    m_operator.get(Button.RB).onFalse(new ParallelCommandGroup(new InstantCommand(()->m_intake.stop()), new ExtendToPosition(m_arm, ArmConstants.kInitialPosition)));

    // Outtakes
    m_operator.get(m_operator.LEFT_TRIGGER_BUTTON).onTrue(new InstantCommand(()->m_intake.intake(-1)));
    m_operator.get(m_operator.LEFT_TRIGGER_BUTTON).onFalse(new InstantCommand(()->m_intake.stop()));

    // Extends the bar
    // These commands don't exist yet
    // m_operator.get(Button.B).onTrue(new ExtendBar());
    // m_operator.get(Button.B).onFalse(new RetractBar());
  }

  public void selectValue(int index, int value){
    selectValues[index] = value;
    selectedNode = new Node(m_vision, DriverStation.getAlliance(), selectValues[1], selectValues[0]*3-3+selectValues[2]);
    System.out.println(selectedNode.scorePose);
}

private final BooleanSupplier LEFT_STICK_LEFT = () -> m_operator.get(Axis.LEFT_X) < -0.75,
LEFT_STICK_RIGHT = () -> m_operator.get(Axis.LEFT_X) > 0.75,
LEFT_STICK_UP = () -> m_operator.get(Axis.LEFT_Y) < -0.75;
private final BooleanSupplier RIGHT_STICK_LEFT = () -> m_operator.get(Axis.RIGHT_X) < -0.75,
RIGHT_STICK_RIGHT = () -> m_operator.get(Axis.RIGHT_X) > 0.75,
RIGHT_STICK_UP = () -> m_operator.get(Axis.RIGHT_Y) < -0.75;


  /**
   * Configures all of the operator controls.
   */
}
