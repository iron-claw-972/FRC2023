package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.BalanceCommand;
import frc.robot.commands.arm.ExtendToPosition;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.OIConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.FourBarArm;
import frc.robot.subsystems.Intake;
import lib.controllers.GameController;
import lib.controllers.GameController.DPad;
public class Operator {

  private GameController operator = new GameController(OIConstants.kOperatorJoy);
  private FourBarArm m_arm;
  private Intake m_intake;
  private Drivetrain m_drive;

  /**
   * Configures all of the operator controls.
   */
  public Operator(FourBarArm arm, Intake intake, Drivetrain drive){
    m_arm = arm;
    m_intake = intake;
    m_drive = drive;
  }

  public void configureControls() {

    // balance command
    operator.get(GameController.Button.RB).onTrue(new PrintCommand("TEST"));
  }

}
