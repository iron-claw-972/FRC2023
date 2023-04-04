package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.scoring.PositionIntake;
import frc.robot.commands.scoring.PositionIntake.Position;
import frc.robot.commands.scoring.elevator.MoveElevator;
import frc.robot.commands.scoring.intake.IntakeGamePiece;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

public class GridNineTwoPiece extends SequentialCommandGroup {
  public GridNineTwoPiece(Drivetrain drive, Elevator elevator, Wrist wrist, Intake intake) {
    addCommands(
      new AutoDeposit(Position.TOP, false, elevator, wrist, intake),
      new PathPlannerCommand("Grid 9 Two Piece", 0, drive, true)
        .alongWith(new PositionIntake(elevator, wrist, () -> false, Position.STOW).andThen(
          new WaitCommand(0.8), 
          Commands.parallel(
            new PositionIntake(elevator, wrist, () -> false, Position.INTAKE), 
            new IntakeGamePiece(intake, () -> false, false))
          )),
      new PathPlannerCommand("Grid 9 Two Piece", 1, drive, true).deadlineWith(Commands.sequence(
        new PositionIntake(elevator, wrist, () -> false, Position.STOW),
        new WaitCommand(1.0),
        new MoveElevator(elevator, ElevatorConstants.kAutoTop))),
      new AutoDeposit(Position.TOP, false, elevator, wrist, intake, () -> false)
    );
  }
}
