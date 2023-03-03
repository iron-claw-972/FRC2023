package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.scoring.PositionIntake;
import frc.robot.commands.scoring.Stow;
import frc.robot.commands.scoring.PositionIntake.Position;
import frc.robot.commands.scoring.bar.CalibrateBar;
import frc.robot.commands.scoring.bar.MoveBar;
import frc.robot.commands.scoring.elevator.CalibrateElevator;
import frc.robot.commands.scoring.intake.IntakeGamePiece;
import frc.robot.commands.scoring.intake.Outtake;
import frc.robot.constants.BarConstants;
import frc.robot.constants.OIConstants;
import frc.robot.subsystems.FourBarArm;
import frc.robot.subsystems.Bar;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Bar.BarPosition;
import lib.controllers.GameController;
import lib.controllers.GameController.Button;
import lib.controllers.GameController.DPad;

public class Operator {

  private GameController m_operator = new GameController(OIConstants.kOperatorJoy);
  
  /**
   * Configures the operator controls for the deploying Bar.
   */
  public void configureControls(FourBarArm arm, Intake intake, Elevator elevator, Bar bar) {
    m_operator.get(DPad.DOWN).onTrue(new CalibrateElevator(elevator));
    m_operator.get(DPad.LEFT).onTrue(new CalibrateBar(bar));

    m_operator.get(DPad.UP).onTrue(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()));

    m_operator.get(Button.Y).onTrue(new PositionIntake(elevator, arm, intake::hasCone, Position.TOP));
    m_operator.get(Button.X).onTrue(new PositionIntake(elevator, arm, intake::hasCone, Position.MIDDLE));
    m_operator.get(Button.A).onTrue(new PositionIntake(elevator, arm, intake::hasCone, Position.INTAKE));
    m_operator.get(Button.B).onTrue(new PositionIntake(elevator, arm, intake::hasCone, Position.SHELF));
    m_operator.get(Button.RB).onTrue(new PositionIntake(elevator, arm, intake::hasCone, Position.STOW));

    m_operator.get(Button.LB).onTrue(new IntakeGamePiece(intake)).onFalse(new Stow(intake, elevator, arm));

    //TODO: cleaner
    new Trigger(m_operator.LEFT_TRIGGER_BUTTON).onTrue(new Outtake(intake, false)).onFalse(new Stow(intake, elevator, arm));
  }
}
