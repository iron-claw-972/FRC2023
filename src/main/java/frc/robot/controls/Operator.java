package frc.robot.controls;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.arm.ExtendToPosition;
import frc.robot.commands.elevator.MoveToExtension;
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

public class Operator {

  private GameController m_operator = new GameController(OIConstants.kOperatorJoy);
  private FourBarArm m_arm;
  private Intake m_intake;

  public Operator(FourBarArm arm, Intake intake){
    m_arm = arm;
    m_intake = intake;
  }

  /**
   * Configures all of the operator controls.
   */
  public void configureControls() {

  }


}
