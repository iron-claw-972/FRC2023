package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.scoring.PositionRollerIntake;
import frc.robot.commands.scoring.PositionRollerIntake.RollerPosition;
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
  public DepositThenPath(String pathName, RollerPosition depositPosition, Drivetrain drive, Elevator elevator, Wrist wrist, RollerIntake intake) {
    addRequirements(drive, elevator, wrist, intake);
    addCommands(
      new InstantCommand(() -> drive.setPigeonYaw(PathGroupLoader.getPathGroup(pathName).get(0))),
      new CalibrateElevator(elevator),
      depositPosition == RollerPosition.TOP ?
        new MoveElevator(elevator, ElevatorConstants.kMiddleConeHeight).withTimeout(1).andThen(new PositionRollerIntake(elevator, wrist, () -> false, RollerPosition.TOP).withTimeout(1.5)) :
        new PositionRollerIntake(elevator, wrist, () -> false, depositPosition).withTimeout(1.5),
      new OuttakeGamePiece(intake, GamePieceType.CONE),
      new PathPlannerCommand(pathName, 0, drive, true),
      new PositionRollerIntake(elevator, wrist, () -> true, RollerPosition.STOW),
      new PathPlannerCommand(pathName, 1, drive, true)
    );
  }
}
