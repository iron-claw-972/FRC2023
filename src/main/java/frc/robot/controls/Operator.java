package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.scoring.Dunk;
import frc.robot.commands.scoring.PositionIntake;
import frc.robot.commands.scoring.PositionIntake.Position;
import frc.robot.commands.scoring.arm.ExtendArm;
import frc.robot.commands.scoring.Stow;
import frc.robot.commands.scoring.bar.CalibrateBar;
import frc.robot.commands.scoring.bar.ToggleBar;
import frc.robot.commands.scoring.elevator.CalibrateElevator;
import frc.robot.commands.scoring.elevator.MoveElevator;
import frc.robot.commands.scoring.intake.IntakeGamePiece;
import frc.robot.commands.scoring.intake.Outtake;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.OIConstants;
import frc.robot.subsystems.Bar;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.FourBarArm;
import frc.robot.subsystems.Intake;
import lib.controllers.GameController;
import lib.controllers.GameController.Button;
import lib.controllers.GameController.DPad;

public class Operator {

  private GameController m_operator = new GameController(OIConstants.kOperatorJoy);
  
  /**
   * Configures the operator controls for the deploying Bar.
   */
  public void configureControls(FourBarArm arm, Intake intake, Elevator elevator, Bar bar) {
    m_operator.get(Button.BACK).onTrue(new CalibrateElevator(elevator));

    if (bar != null) {
      m_operator.get(DPad.LEFT).onTrue(new CalibrateBar(bar));
      new Trigger(m_operator.RIGHT_TRIGGER_BUTTON).onTrue(new ToggleBar(bar));
    }

    

    m_operator.get(DPad.UP).onTrue(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()));

    //top
    m_operator.get(Button.Y).onTrue(new PositionIntake(elevator, arm, intake::hasCone, Position.TOP));
    //middle
    m_operator.get(Button.X).onTrue(new PositionIntake(elevator, arm, intake::hasCone, Position.MIDDLE));
    //bottom
    m_operator.get(Button.A).onTrue(new PositionIntake(elevator, arm, intake::hasCone, Position.BOTTOM));
    //shelf
    m_operator.get(Button.B).onTrue(new PositionIntake(elevator, arm, intake::hasCone, Position.SHELF).alongWith(new IntakeGamePiece(intake)))
      .onFalse(new SequentialCommandGroup( 
        new InstantCommand(() -> intake.stopIntake()),
        new ExtendArm(arm, 0.8),
        new MoveElevator(elevator, ElevatorConstants.kStowHeight),
        new Stow(intake, elevator, arm)
      ));
    
    //stow
    m_operator.get(Button.RB).onTrue(new Stow(intake, elevator, arm));

    //intake
    m_operator.get(Button.LB).onTrue(
      new PositionIntake(elevator, arm, intake::hasCone, Position.INTAKE).alongWith(new IntakeGamePiece(intake)))
      .onFalse(new Stow(intake, elevator, arm));

    //dunk
    m_operator.get(Button.RIGHT_JOY).onTrue(new Dunk(arm, intake));

    //TODO: cleaner
    new Trigger(m_operator.LEFT_TRIGGER_BUTTON).onTrue(new Outtake(intake, false)).onFalse(new Stow(intake, elevator, arm));
  }
}
