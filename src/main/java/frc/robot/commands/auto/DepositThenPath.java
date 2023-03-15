package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.scoring.PositionIntake;
import frc.robot.commands.scoring.PositionIntake.Position;
import frc.robot.commands.scoring.elevator.CalibrateElevator;
import frc.robot.commands.scoring.elevator.MoveElevator;
import frc.robot.commands.scoring.intake.OuttakeGamePiece;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.RollerIntake;
import frc.robot.subsystems.Wrist;
import frc.robot.util.GamePieceType;

public class DepositThenPath extends SequentialCommandGroup {
  public DepositThenPath(String pathName, Position depositPosition, Drivetrain drive, Elevator elevator, Wrist wrist, RollerIntake intake) {
    addRequirements(drive, elevator, wrist, intake);
    addCommands(
      new CalibrateElevator(elevator),
      depositPosition == Position.TOP ?
        new MoveElevator(elevator, ElevatorConstants.kMiddleConeHeight).withTimeout(1).andThen(new PositionIntake(elevator, wrist, () -> false, Position.TOP).withTimeout(1.5)) :
        new PositionIntake(elevator, wrist, () -> false, depositPosition).withTimeout(1.5),
      new OuttakeGamePiece(intake, GamePieceType.CONE),
      new PathPlannerCommand(pathName, 0, drive, true),
      new PositionIntake(elevator, wrist, () -> true, Position.STOW),
      new PathPlannerCommand(pathName, 1, drive, false)
    );
  }
}
