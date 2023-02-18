package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.arm.ExtendToPosition;
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
import lib.controllers.GameController.Button;
import lib.controllers.GameController.DPad;
import lib.controllers.GameController.Axis;;

public class Operator {
  private static GameController operator = new GameController(OIConstants.kOperatorJoy);

  public static void configureControls(FourBarArm m_arm, Intake m_intake, Elevator m_elevator) {
    //TODO: change keybinds so that the same one doesn't call two things. This may or may not be needed to be done. 
    
    //Move to max height
    operator.get(operator.LEFT_TRIGGER_BUTTON).onTrue(new MoveToHeight(m_elevator, ElevatorConstants.kMaxHeight)); 
    //Move to min height
    operator.get(operator.RIGHT_TRIGGER_BUTTON).onTrue(new MoveToHeight(m_elevator, ElevatorConstants.kBottomHeight)); 
    //Calibrate elevator using inbuilt motor encoders
    operator.get(DPad.DOWN).onTrue(new ResetEncoderAtBottom(m_elevator));
    //TODO: calibrate elevator using absolute encoders(probably will not work yet as of 2/15/2023);
    //move to bottom node height
    operator.get(Button.A).onTrue(new MoveToHeight(m_elevator, ElevatorConstants.kHeightBottomNode));
    //move to mid node height
    operator.get(Button.B).onTrue(new MoveToHeight(m_elevator, ElevatorConstants.kHeightMiddleNode));
    //move to top node height
    operator.get(Button.Y).onTrue(new MoveToHeight(m_elevator, ElevatorConstants.kHeightTopNode));    
    // arm controls
    operator.get(Button.Y).onTrue(new ExtendToPosition(m_arm, ArmConstants.kTopPosition));
    operator.get(Button.X).onTrue(new ExtendToPosition(m_arm, ArmConstants.kMiddlePosition));
    operator.get(Button.A).onTrue(new ExtendToPosition(m_arm, ArmConstants.kIntakePosition));
    operator.get(Button.B).onTrue(new ExtendToPosition(m_arm, ArmConstants.kShelfPosition));
    // intake controls
    operator.get(DPad.DOWN).onTrue(new InstantCommand(() -> m_intake.intake(IntakeConstants.kIntakeSpeed), m_intake));
    operator.get(DPad.UP).onTrue(new InstantCommand(() -> m_intake.intake(IntakeConstants.kOuttakeSpeed),m_intake));
    operator.get(DPad.LEFT).onTrue(new InstantCommand(() -> m_intake.stop(), m_intake));
  }

  public static double getRawThrottleValue() {
    return operator.get(Axis.LEFT_Y);
  }
}
