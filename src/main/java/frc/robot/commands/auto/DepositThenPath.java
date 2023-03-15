package frc.robot.commands.auto;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.scoring.PositionIntake;
import frc.robot.commands.scoring.PositionIntake.Position;
import frc.robot.commands.scoring.elevator.CalibrateElevator;
import frc.robot.commands.scoring.elevator.MoveElevator;
import frc.robot.commands.scoring.intake.OuttakeGamePiece;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.FourBarArm;
import frc.robot.subsystems.RollerIntake;
import frc.robot.util.GamePieceType;
import frc.robot.util.PathGroupLoader;

public class DepositThenPath extends SequentialCommandGroup {

  public DepositThenPath(String pathName, Position depositPosition, Drivetrain drive, Elevator elevator, FourBarArm arm, RollerIntake intake) {
    this(pathName, depositPosition, new Pose2d(), drive, elevator, arm, intake);
  }

  public DepositThenPath(String pathName, Position depositPosition, Pose2d offset, Drivetrain drive, Elevator elevator, FourBarArm arm, RollerIntake intake) {
    addRequirements(drive, elevator, arm, intake);

    List<PathPlannerTrajectory> pathsOld = PathGroupLoader.getPathGroup(pathName);
    List<PathPlannerTrajectory> paths = new ArrayList<>();

    // TODO: test this works. casting may cause issues
    for (int i = 0; i < pathsOld.size(); i++) {
      paths.add((PathPlannerTrajectory) pathsOld.get(i).relativeTo(offset.times(-1)));
    }

    addCommands(
      new CalibrateElevator(elevator),
      depositPosition == Position.MIDDLE ?
        new PositionIntake(elevator, arm, () -> false, depositPosition).withTimeout(1.5) : 
        new MoveElevator(elevator, ElevatorConstants.kMiddleConeHeight).withTimeout(1).andThen(new PositionIntake(elevator, arm, () -> false, Position.TOP).withTimeout(1.5)),
      new OuttakeGamePiece(intake, GamePieceType.CONE),
      new PathPlannerCommand(paths, 0, drive, true),
      new PositionIntake(elevator, arm, () -> true, Position.STOW),
      new PathPlannerCommand(paths, 1, drive, true)
    );
  }
}
