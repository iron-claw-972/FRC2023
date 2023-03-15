package frc.robot.commands.auto;

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
import frc.robot.subsystems.FourBarArm;
import frc.robot.subsystems.RollerIntake;
import frc.robot.util.GamePieceType;
import frc.robot.util.PathGroupLoader;

public class DepositThenPath extends SequentialCommandGroup {
  public DepositThenPath(String pathName, Position depositPosition, Drivetrain drive, Elevator elevator, FourBarArm arm, RollerIntake intake) {
    addRequirements(drive, elevator, arm, intake);
    addCommands(
      new InstantCommand(() -> drive.setPigeonYaw(PathGroupLoader.getPathGroup(pathName).get(0))),
      new CalibrateElevator(elevator),
      depositPosition == Position.MIDDLE ?
        new PositionIntake(elevator, arm, () -> false, depositPosition).withTimeout(1.5) : 
        new MoveElevator(elevator, ElevatorConstants.kMiddleConeHeight).withTimeout(1).andThen(new PositionIntake(elevator, arm, () -> false, Position.TOP).withTimeout(1.5)),
      new OuttakeGamePiece(intake, GamePieceType.CONE),
      new PathPlannerCommand(pathName, 0, drive, true),
      new PositionIntake(elevator, arm, () -> true, Position.STOW),
      new PathPlannerCommand(pathName, 1, drive, true)
    );
  }
}
