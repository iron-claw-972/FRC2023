package frc.robot.commands.auto;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
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
   * Deposit a game piece in the given row during auto. This command assumes that the robot is already in the correct column position and that the intake is already holding a game piece. This will not automatically detect the game piece. It will assume that the game piece is a cone.
   * @param depositPosition
   * @param elevator
   * @param wrist
   * @param intake
   */
  public AutoDeposit(Position depositPosition, Elevator elevator, Wrist wrist, RollerIntake intake) {
    this(depositPosition, elevator, wrist, intake, () -> true);
  }

  /**
   * Deposit a game piece in the given row during auto. This command assumes that the robot is already in the correct column position and that the intake is already holding a game piece.
   * 
   * @param depositPosition the row to deposit the game piece in
   * @param elevator the elevator subsystem
   * @param wrist the wrist subsystem
   * @param intake the intake subsystem
   * @param isCone a boolean supplier that returns true if the intake is holding a cone, false if cube
   */
  public AutoDeposit(Position depositPosition, Elevator elevator, Wrist wrist, RollerIntake intake, BooleanSupplier isCone) {
    addRequirements(elevator, wrist, intake);

    Command depositCommand;

    // TODO: Add elevator and wrist constants for cube deposit positions
    if (depositPosition == Position.TOP) {
      depositCommand = new MoveElevator(elevator, ElevatorConstants.kAutoTop).andThen(new RotateWrist(wrist, WristConstants.kAutoTop));
    } else if (depositPosition == Position.MIDDLE) {
      depositCommand = new MoveElevator(elevator, ElevatorConstants.kAutoMiddle).andThen(new RotateWrist(wrist, WristConstants.kAutoMiddle));
    } else {
      // If hybrid, just shooting the came piece will make it land in the node
      depositCommand = new DoNothing();
    }

    addCommands(
      new ConditionalCommand(new DoNothing(), new CalibrateElevator(elevator), () -> elevator.isCalibrated()),
      depositCommand,
      new WaitCommand(0.25),
      new OuttakeGamePiece(intake, () -> (isCone.getAsBoolean() ? GamePieceType.CONE : GamePieceType.CUBE)),
      new PositionIntake(elevator, wrist, isCone, Position.STOW)
    );
  }
}
