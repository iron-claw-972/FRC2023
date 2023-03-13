package frc.robot.controls;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DepositTune;
import frc.robot.commands.scoring.Stow;
import frc.robot.commands.scoring.bar.CalibrateBar;
import frc.robot.commands.scoring.elevator.CalibrateElevator;
import frc.robot.commands.scoring.intake.IntakeGamePiece;
import frc.robot.commands.scoring.intake.OuttakeGamePiece;
import frc.robot.commands.scoring.wrist.RotateWrist;
import frc.robot.constants.OIConstants;
import frc.robot.constants.WristConstants;
import frc.robot.subsystems.Bar;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.FourBarArm;
import frc.robot.subsystems.RollerIntake;
import frc.robot.subsystems.RollerIntake.IntakeMode;
import frc.robot.subsystems.RollerIntake.IntakePiece;
import frc.robot.subsystems.Wrist;
import lib.controllers.GameController;
import lib.controllers.GameController.Button;
import lib.controllers.GameController.DPad;

public class TestController {
  
  private GameController test = new GameController(OIConstants.kTestJoy);
  private FourBarArm m_arm;
  private RollerIntake m_intake;
  private Elevator m_elevator;
  private Bar m_bar;
  private Wrist m_wrist;
  
  public TestController(FourBarArm arm, RollerIntake intake, Elevator elevator, Bar bar, Wrist wrist) {
    m_arm = arm;
    m_intake = intake;
    m_elevator = elevator;
    m_bar = bar;
    m_wrist = wrist;
  }
  
  public void configureControls() {

    test.get(DPad.DOWN).onTrue(new CalibrateElevator(m_elevator));

    // SmartDashboard.putNumber("Deposit Elevator Extension", 0);
    // SmartDashboard.putNumber("Deposit Arm Extension", 0);

    test.get(DPad.UP).onTrue(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()));

    /*test.get(Button.A).onTrue(new IntakeGamePiece(m_intake, IntakePiece.CONE));
    test.get(Button.B).onTrue(new IntakeGamePiece(m_intake, IntakePiece.CUBE));
    test.get(Button.X).onTrue(new OuttakeGamePiece(m_intake));
    test.get(Button.Y).onTrue(new InstantCommand(() -> m_intake.setIntakeMode(IntakeMode.DISABLED), m_intake));
    */
    // test.get(Button.Y).onTrue(new MoveElevator(m_elevator, ElevatorConstants.kTopConeHeight));
    // test.get(Button.X).onTrue(new MoveElevator(m_elevator, ElevatorConstants.kMiddleConeHeight));
    // test.get(Button.A).onTrue(new MoveElevator(m_elevator, ElevatorConstants.kBottomConeHeight));
    // test.get(Button.B).onTrue(new MoveElevator(m_elevator, ElevatorConstants.kShelfConeHeight));
    // test.get(Button.RB).onTrue(new MoveElevator(m_elevator, ElevatorConstants.kIntakeConeHeight));

    // test.get(Button.Y).onTrue(new ExtendArm(m_arm, ArmConstants.kTopNodePos));
    // test.get(Button.X).onTrue(new ExtendArm(m_arm, ArmConstants.kMiddleNodePos));
    // test.get(Button.A).onTrue(new ExtendArm(m_arm, ArmConstants.kBottomNodePos));
    // test.get(Button.B).onTrue(new ExtendArm(m_arm, ArmConstants.kShelfPos));
    // test.get(Button.RB).onTrue(new ExtendArm(m_arm, ArmConstants.kIntakePos));

    test.get(Button.Y).onTrue(new RotateWrist(m_wrist, WristConstants.kTopNodePos));
    test.get(Button.X).onTrue(new RotateWrist(m_wrist, WristConstants.kMiddleNodePos));
    test.get(Button.A).onTrue(new RotateWrist(m_wrist, WristConstants.kBottomNodePos));
    test.get(Button.B).onTrue(new RotateWrist(m_wrist, WristConstants.kShelfPos));
    test.get(Button.RB).onTrue(new RotateWrist(m_wrist, WristConstants.kIntakeConePos));
    test.get(Button.LB).onTrue(new RotateWrist(m_wrist, WristConstants.kIntakeCubePos));
    test.get(test.RIGHT_TRIGGER_BUTTON).onTrue(new RotateWrist(m_wrist, WristConstants.kStowPos));


    // test.get(Button.LB).onTrue(new IntakeGamePiece(m_intake));

    // test.get(Button.RB).onTrue(new OuttakeGamePiece(m_intake, false));

    //TODO: cleaner
    // new Trigger(test.LEFT_TRIGGER_BUTTON).onTrue(new Stow(m_intake, m_elevator, m_arm));

    // test.get(DPad.LEFT).onTrue(new DepositMiddleNode(m_elevator, m_arm, m_intake));
    // test.get(DPad.RIGHT).onTrue(new DepositTopNode(m_elevator,m_arm,m_intake));
    // test.get(DPad.UP).onTrue(new ExtendArm(m_arm, ArmConstants.kStowedAbsEncoderPos));
    //test.get(Button.B).onTrue(new ExtendToPosition(m_arm, ArmConstants.kShelfPositionAbsEncoderPos));
  
  
    //test.get(DPad.DOWN).onTrue(new InstantCommand(() -> m_intake.intake(IntakeConstants.kIntakeSpeed), m_intake));
    //test.get(DPad.UP).onTrue(new InstantCommand(() -> m_intake.intake(IntakeConstants.kOuttakeSpeed),m_intake));
    //test.get(DPad.LEFT).onTrue(new InstantCommand(() -> m_intake.stopIntake(), m_intake));


    //test.get(DPad.LEFT).onTrue(new InstantCommand(() -> m_elevator.setMotorPower(0), m_elevator));
    // test.get(DPad.DOWN).onTrue(new CalibrateElevator(m_elevator));
    // test.get(Button.B).onTrue(new MoveElevator(m_elevator, ElevatorConstants.kMiddleConeHeight));
    // test.get(Button.X).onTrue(new MoveElevator(m_elevator, ElevatorConstants.kTopConeHeight));
    // test.get(Button.Y).onTrue(new MoveElevator(m_elevator, ElevatorConstants.kBottomConeHeight));
    
  }
}