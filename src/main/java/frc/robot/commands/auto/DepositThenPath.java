package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DoNothing;
import frc.robot.commands.scoring.PositionIntake;
import frc.robot.commands.scoring.PositionIntake.Position;
import frc.robot.commands.scoring.elevator.CalibrateElevator;
import frc.robot.commands.scoring.elevator.MoveElevator;
import frc.robot.commands.scoring.intake.OuttakeGamePiece;
import frc.robot.commands.scoring.wrist.RotateWrist;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.WristConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;
import frc.robot.util.GamePieceType;

public class DepositThenPath extends SequentialCommandGroup {

  // TODO: refactor this to use seperate deposit Auto commands, and allow depositing Cubes
  public DepositThenPath(String pathName, Position depositPosition, boolean doesPath, Drivetrain drive, Elevator elevator, Wrist wrist, Intake intake) {
    addRequirements(drive, elevator, wrist, intake);

    Command depositCommand;

    if (depositPosition == Position.TOP) {
      depositCommand = new MoveElevator(elevator, ElevatorConstants.kAutoTop).andThen(new RotateWrist(wrist, WristConstants.kAutoTop));
    } else if (depositPosition == Position.MIDDLE) {
      depositCommand = new MoveElevator(elevator, ElevatorConstants.kAutoMiddle).andThen(new RotateWrist(wrist, WristConstants.kAutoMiddle));
    } else {
      depositCommand = new DoNothing();
    }

    addCommands(
      new CalibrateElevator(elevator),
      depositCommand,
      new WaitCommand(0.25),
      new OuttakeGamePiece(intake, () -> GamePieceType.CONE),
      new PositionIntake(elevator, wrist, () -> true, Position.STOW),
      doesPath ? new PathPlannerCommand(pathName, 0, drive, true) : new DoNothing()
    );
  }
}
