package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.scoring.PositionIntake;
import frc.robot.commands.scoring.Stow;
import frc.robot.commands.scoring.PositionIntake.Position;
import frc.robot.commands.scoring.elevator.MoveElevator;
import frc.robot.commands.scoring.intake.IntakeGamePiece;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;
import frc.robot.util.GamePieceType;

public class GridNineTwoPiece extends SequentialCommandGroup {

  /**
   * Runs an auto from grid 9 (closest to the loading zone) that deposits the preloaded game piece, drives to 
   * the staged game piece nearest to grid 9, then scores that one. Does not stow at the end.
   * @param drive
   * @param elevator
   * @param wrist
   * @param intake
   */
  public GridNineTwoPiece(Drivetrain drive, Elevator elevator, Wrist wrist, Intake intake) {
    addCommands(
      // start by depositing in the top. Do not stow, will run in parallel for speed
      new AutoDeposit(Position.TOP, GamePieceType.CONE, false, elevator, wrist, intake),
      // drive to second GP...
      new PathPlannerCommand("Grid 9 Two Piece", 0, drive, true)
        // In parallel,
        .alongWith(
          // stow, and then after 0.8 sec put the intake down and spin the intake for cube
          new Stow(elevator, wrist).andThen(
          new WaitCommand(0.8), 
          Commands.parallel(
            new PositionIntake(elevator, wrist, GamePieceType.CUBE, Position.INTAKE), 
            // note false for runs forever, will wait until detects cube in intake
            new IntakeGamePiece(intake, GamePieceType.CUBE, false)
          )
        )),
      // after intake has finished, drive back to grid
      new PathPlannerCommand("Grid 9 Two Piece", 1, drive, false)
        // while driving back to grid, first stow then start moving to score position
        // not vital that we move to score position, so it is a deadline group -- if the drive command finishes, so do 
        // the commands moving the elevator. AutoDeposit() will start as soon as it finishes driving, which will move the subsystems
        .deadlineWith(Commands.sequence(
          new Stow(elevator, wrist),
          new WaitCommand(1.0),
          new MoveElevator(elevator, ElevatorConstants.kAutoTop)
        )),
      // And finally, score GP 2! It will not stow, for compatibility with later autos...
      new AutoDeposit(Position.TOP, GamePieceType.CUBE, false, elevator, wrist, intake)
    );
  }
}
