package frc.robot.commands.auto;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DoNothing;
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
import frc.robot.util.PathGroupLoader;

public class DepositThenPath extends SequentialCommandGroup {

  public DepositThenPath(String pathName, Position depositPosition, Drivetrain drive, Elevator elevator, Wrist wrist, RollerIntake intake) {
    this(pathName, depositPosition, new Pose2d(), drive, elevator, wrist, intake);
  }

  // TODO: refactor this to use seperate deposit Auto commands, and allow depositing Cubes
  public DepositThenPath(String pathName, Position depositPosition, Pose2d offset, Drivetrain drive, Elevator elevator, Wrist wrist, RollerIntake intake) {
    addRequirements(drive, elevator, wrist, intake);

    List<PathPlannerTrajectory> pathsOld = PathGroupLoader.getPathGroup(pathName);
    List<PathPlannerTrajectory> paths = new ArrayList<>();

    // TODO: test this works. casting may cause issues
    for (int i = 0; i < pathsOld.size(); i++) {
      paths.add((PathPlannerTrajectory) pathsOld.get(i).relativeTo(offset.times(-1)));
    }

    addCommands(
      new CalibrateElevator(elevator),
      depositPosition == Position.TOP ?
        // For top node, need to move the elevator first so that wrist doesn't hit nodes
        new MoveElevator(elevator, ElevatorConstants.kMiddleConeHeight).withTimeout(1) : new DoNothing(),
      new PositionIntake(elevator, wrist, () -> true, depositPosition).withTimeout(1.5),
      new OuttakeGamePiece(intake, () -> GamePieceType.CONE),
      new PositionIntake(elevator, wrist, () -> true, Position.STOW),
      new PathPlannerCommand(paths, 0, drive, true)
    );
  }
}
