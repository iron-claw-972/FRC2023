package frc.robot.commands.auto;

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
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.RollerIntake;
import frc.robot.subsystems.Wrist;
import frc.robot.util.GamePieceType;

public class AutoDeposit extends SequentialCommandGroup {

  /**
   * Deposit a game piece in the given row during auto. This command assumes that the robot is already in the correct column position and that the intake is already holding a game piece.
   * 
   * @param depositPosition the row to deposit the game piece in
   * @param elevator the elevator subsystem
   * @param wrist the wrist subsystem
   * @param intake the intake subsystem
   */
  public AutoDeposit(Position depositPosition, Elevator elevator, Wrist wrist, RollerIntake intake) {
    addRequirements(elevator, wrist, intake);

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
      new PositionIntake(elevator, wrist, () -> true, Position.STOW)
    );
  }
}
