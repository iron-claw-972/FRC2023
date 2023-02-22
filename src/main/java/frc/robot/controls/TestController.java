package frc.robot.controls;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.arm.ExtendToPosition;
import frc.robot.commands.elevator.GetMaxHeight;
import frc.robot.commands.elevator.MoveToHeight;
import frc.robot.commands.elevator.ResetEncoderAtBottom;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.OIConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.FourBarArm;
import frc.robot.subsystems.Intake;
import lib.controllers.GameController;
import lib.controllers.GameController.Axis;
import lib.controllers.GameController.Button;
import lib.controllers.GameController.DPad;

public class TestController {
  
  private GameController test = new GameController(OIConstants.kTestJoy);
  private FourBarArm m_arm;
  private Intake m_intake;
  private Elevator m_elevator;
  
  public TestController(FourBarArm arm, Intake intake, Elevator elevator){
    m_arm = arm;
    m_intake = intake;
    m_elevator = elevator;
  }
  
  public void configureControls() {
    
    // elevator controls
    if (m_arm != null) {
      test.get(Button.Y).onTrue(new ExtendToPosition(m_arm, ArmConstants.kTopPosition));
      test.get(Button.X).onTrue(new ExtendToPosition(m_arm, ArmConstants.kMiddlePosition));
      test.get(Button.A).onTrue(new ExtendToPosition(m_arm, ArmConstants.kIntakePosition));
      test.get(Button.B).onTrue(new ExtendToPosition(m_arm, ArmConstants.kShelfPosition));
    }
    
    // intake controls
    if (m_intake != null) {
      test.get(DPad.DOWN).onTrue(new InstantCommand(() -> m_intake.intake(IntakeConstants.kIntakeSpeed), m_intake));
      test.get(DPad.UP).onTrue(new InstantCommand(() -> m_intake.intake(IntakeConstants.kOuttakeSpeed),m_intake));
      test.get(DPad.LEFT).onTrue(new InstantCommand(() -> m_intake.stop(), m_intake));
    }
    
    // elevator controls
    if (m_elevator != null) {
      //test.get(DPad.DOWN).onTrue(new InstantCommand(() -> m_elevator.setMotorPower(-0.1), m_elevator));
      //test.get(DPad.UP).onTrue(new InstantCommand(() -> m_elevator.setMotorPower(0.1),m_elevator));
      test.get(DPad.LEFT).onTrue(new InstantCommand(() -> m_elevator.setMotorPower(0), m_elevator));
      test.get(DPad.DOWN).onTrue(new ResetEncoderAtBottom(m_elevator));
      test.get(DPad.UP).onTrue(new GetMaxHeight(m_elevator));
    
      test.get(Button.A).onTrue(new MoveToHeight(m_elevator, ElevatorConstants.kHeightBottomNode));
      //move to mid node height
      test.get(Button.B).onTrue(new MoveToHeight(m_elevator, ElevatorConstants.kHeightMiddleNode));
      //move to top node height
      test.get(Button.Y).onTrue(new MoveToHeight(m_elevator, ElevatorConstants.kHeightTopNode));   
      

    }
    
  }
  public double getClampedThrottleValue() {
    
    return MathUtil.clamp(test.get(Axis.LEFT_Y),-ElevatorConstants.kPowerLimit, ElevatorConstants.kPowerLimit);
  }
  

}

