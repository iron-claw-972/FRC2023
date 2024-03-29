package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.scoring.PositionIntake;
import frc.robot.commands.scoring.Stow;
import frc.robot.commands.scoring.PositionIntake.Position;
import frc.robot.commands.scoring.elevator.MoveElevator;
import frc.robot.commands.scoring.intake.IntakeGamePiece;
import frc.robot.commands.scoring.wrist.RotateWrist;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.WristConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;
import frc.robot.util.GamePieceType;

public class TwoPiece extends SequentialCommandGroup {

  /**
   * Runs an auto from grid 1 or 9 (closest to the field boundary/loading zone) that deposits the preloaded game piece, drives to 
   * the staged game piece nearest, then scores that one. Does not stow at the end.
   * @param isGrid9 if it should run the grid 9 path or the grid 1 path
   * @param drive
   * @param elevator
   * @param wrist
   * @param intake
   */
  public TwoPiece(boolean isGrid9, Drivetrain drive, Elevator elevator, Wrist wrist, Intake intake) {
    
    String path = "Grid 9 Two Piece";
    if (!isGrid9) {
      path = "Grid 1 Two Piece";
    }

    addCommands(
      // start by depositing in the top. Do not stow, will run in parallel for speed
      new AutoDeposit(Position.TOP, GamePieceType.CONE, false, elevator, wrist, intake),
      // drive to second GP...
      new PathPlannerCommand(path, 0, drive, true)
        // In parallel,
        .alongWith(
          // stow, and then after 0.8 sec put the intake down and spin the intake for cube
          new Stow(elevator, wrist).andThen(
          new WaitCommand(0.8), 
          Commands.parallel(
            new PositionIntake(elevator, wrist, GamePieceType.CUBE, Position.INTAKE), 
            // note false for runs forever, will wait until detects cube in intake or 3 seconds pass
            new IntakeGamePiece(intake, GamePieceType.CUBE, false).withTimeout(3)
          )
        )),
      // after intake has finished, drive back to grid
      new PathPlannerCommand(path, 1, drive, false)
        // while driving back to grid, first stow then start moving to score position
        // not vital that we move to score position, so it is a deadline group -- if the drive command finishes, so do 
        // the commands moving the elevator. AutoDeposit() will start as soon as it finishes driving, which will move the subsystems
        .deadlineWith(Commands.sequence(
          new Stow(elevator, wrist),
          // if in grid 1, it shouldn't extend the elevator/wrist as early 
          // because the robot can tip on the cable tray
          new WaitCommand(isGrid9 ? 1.0 : 3.0),
          new MoveElevator(elevator, ElevatorConstants.kTopCubeHeight).alongWith(new RotateWrist(wrist, WristConstants.kTopNodeCubePos))
        )),
      // And finally, score GP 2! It will not stow, for compatibility with later autos...
      new AutoDeposit(Position.TOP, GamePieceType.CUBE, false, elevator, wrist, intake)
    );
  }
}
