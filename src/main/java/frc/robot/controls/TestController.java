package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.scoring.PositionIntake;
import frc.robot.commands.scoring.PositionIntake.Position;
import frc.robot.commands.scoring.elevator.CalibrateElevator;
import frc.robot.commands.scoring.wrist.RotateWrist;
import frc.robot.constants.OIConstants;
import frc.robot.constants.WristConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.RollerIntake;
import frc.robot.subsystems.RollerIntake.IntakeMode;
import frc.robot.subsystems.Wrist;
import lib.controllers.GameController;
import lib.controllers.GameController.Button;
import lib.controllers.GameController.DPad;

public class TestController {
  
  private GameController test = new GameController(OIConstants.kTestJoy);
  private RollerIntake m_intake;
  private Elevator m_elevator;
  private Wrist m_wrist;
  
  public TestController(Wrist wrist, RollerIntake intake, Elevator elevator) {
    m_intake = intake;
    m_elevator = elevator;
    m_wrist = wrist;
  }
  
  public void configureControls() {

    test.get(DPad.DOWN).onTrue(new CalibrateElevator(m_elevator));

    test.get(DPad.UP).onTrue(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()));

    // INTAKE CONTROLS
    //intake cube
    test.get(Button.RB).onTrue(new InstantCommand(() -> m_intake.setMode(IntakeMode.INTAKE_CUBE), m_intake)).onFalse(new InstantCommand(() -> m_intake.setMode(IntakeMode.DISABLED), m_intake));
    //intake cone
    test.get(Button.LB).onTrue(new InstantCommand(() -> m_intake.setMode(IntakeMode.INTAKE_CONE), m_intake)).onFalse(new InstantCommand(() -> m_intake.setMode(IntakeMode.DISABLED), m_intake));
    //outtake cube
    test.get(test.RIGHT_TRIGGER_BUTTON).onTrue(new InstantCommand(() -> m_intake.setMode(IntakeMode.OUTTAKE_CUBE), m_intake)).onFalse(new InstantCommand(() -> m_intake.setMode(IntakeMode.DISABLED), m_intake));
    //outtake cone
    test.get(test.LEFT_TRIGGER_BUTTON).onTrue(new InstantCommand(() -> m_intake.setMode(IntakeMode.OUTTAKE_CONE), m_intake)).onFalse(new InstantCommand(() -> m_intake.setMode(IntakeMode.DISABLED), m_intake));
    
    // test.get(DPad.LEFT).onTrue(new IntakeGamePiece(m_intake, GamePieceType.CONE));
    // test.get(Button.LEFT_JOY).onTrue(new IntakeGamePiece(m_intake, GamePieceType.CUBE));
    // test.get(DPad.RIGHT).onTrue(new OuttakeGamePiece(m_intake));

    // ELEVATOR CONTROLS
    // test.get(Button.Y).onTrue(new MoveElevator(m_elevator, ElevatorConstants.kTopConeHeight));
    // test.get(Button.X).onTrue(new MoveElevator(m_elevator, ElevatorConstants.kMiddleConeHeight));
    // test.get(Button.A).onTrue(new MoveElevator(m_elevator, ElevatorConstants.kBottomConeHeight));
    // test.get(Button.B).onTrue(new MoveElevator(m_elevator, ElevatorConstants.kShelfConeHeight));
    // test.get(Button.RIGHT_JOY).onTrue(new MoveElevator(m_elevator, ElevatorConstants.kIntakeConeHeight));

    // WRIST CONTROLS
    test.get(Button.Y).onTrue(new RotateWrist(m_wrist, WristConstants.kTopNodeCubePos));
    test.get(Button.X).onTrue(new RotateWrist(m_wrist, WristConstants.kMiddleNodeCubePos));
    test.get(Button.A).onTrue(new RotateWrist(m_wrist, WristConstants.kBottomNodeCubePos));
    test.get(DPad.RIGHT).onTrue(new RotateWrist(m_wrist, WristConstants.kIntakeConePos));
    test.get(DPad.LEFT).onTrue(new RotateWrist(m_wrist, WristConstants.kIntakeCubePos));
    test.get(Button.RIGHT_JOY).onTrue(new RotateWrist(m_wrist, WristConstants.kStowPos));

    // stow
    test.get(Button.BACK).onTrue(new PositionIntake(m_elevator, m_wrist, test.RIGHT_TRIGGER_BUTTON, Position.STOW));
  }
}