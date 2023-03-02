package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.DepositTune;
import frc.robot.commands.arm.ExtendArm;
import frc.robot.commands.elevator.CalibrateElevator;
import frc.robot.commands.elevator.MoveElevator;
import frc.robot.commands.gamePiecePlacement.DepositMiddleNode;
import frc.robot.commands.gamePiecePlacement.DepositTopNode;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.OIConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.FourBarArm;
import frc.robot.subsystems.Intake;
import lib.controllers.GameController;
import lib.controllers.GameController.Button;
import lib.controllers.GameController.DPad;

public class TestController {
  
  private GameController test = new GameController(OIConstants.kTestJoy);
  private FourBarArm m_arm;
  private Intake m_intake;
  private Elevator m_elevator;
  
  public TestController(FourBarArm arm, Intake intake, Elevator elevator) {
    m_arm = arm;
    m_intake = intake;
    m_elevator = elevator;
  }
  
  public void configureControls() {
      test.get(DPad.LEFT).onTrue(new DepositMiddleNode(m_elevator, m_arm, m_intake));
      test.get(DPad.RIGHT).onTrue(new DepositTopNode(m_elevator,m_arm,m_intake));
      test.get(DPad.UP).onTrue(new ExtendArm(m_arm, ArmConstants.kStowedAbsEncoderPos));
      //test.get(Button.B).onTrue(new ExtendToPosition(m_arm, ArmConstants.kShelfPositionAbsEncoderPos));
    
   
      //test.get(DPad.DOWN).onTrue(new InstantCommand(() -> m_intake.intake(IntakeConstants.kIntakeSpeed), m_intake));
      //test.get(DPad.UP).onTrue(new InstantCommand(() -> m_intake.intake(IntakeConstants.kOuttakeSpeed),m_intake));
      //test.get(DPad.LEFT).onTrue(new InstantCommand(() -> m_intake.stopIntake(), m_intake));
  

      //test.get(DPad.LEFT).onTrue(new InstantCommand(() -> m_elevator.setMotorPower(0), m_elevator));
      test.get(DPad.DOWN).onTrue(new CalibrateElevator(m_elevator));
      test.get(Button.B).onTrue(new MoveElevator(m_elevator, ElevatorConstants.kMiddleNodeHeightExtension));
      test.get(Button.X).onTrue(new MoveElevator(m_elevator, ElevatorConstants.kTopNodeHeightExtension));
      test.get(Button.Y).onTrue(new MoveElevator(m_elevator, ElevatorConstants.kMinExtension));
    
  }
}