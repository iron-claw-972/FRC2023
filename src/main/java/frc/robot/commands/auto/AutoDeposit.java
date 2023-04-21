package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DoNothing;
import frc.robot.commands.scoring.PositionIntake;
import frc.robot.commands.scoring.Stow;
import frc.robot.commands.scoring.PositionIntake.Position;
import frc.robot.commands.scoring.elevator.CalibrateElevator;
import frc.robot.commands.scoring.elevator.MoveElevator;
import frc.robot.commands.scoring.intake.OuttakeGamePiece;
import frc.robot.commands.scoring.wrist.RotateWrist;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.WristConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;
import frc.robot.util.GamePieceType;

public class AutoDeposit extends SequentialCommandGroup {

  /**
   * Deposit a game piece in the given row during auto. 
   * This command assumes that the robot is already in the correct column position and that the intake is already holding a game piece. 
   * This will not automatically detect the game piece. It will assume that the game piece is a cone.
   * @param depositPosition which position to score in, bottom (hybrid), middle, or top. 
   * @param stows stows after running
   * @param elevator
   * @param wrist
   * @param intake
   */
  public AutoDeposit(Position depositPosition, boolean stows, Elevator elevator, Wrist wrist, Intake intake) {
    this(depositPosition, GamePieceType.CONE, stows, elevator, wrist, intake);
  }

  /**
   * Deposit a game piece in the given row during auto. This command assumes that the robot is already in the correct column position and that the intake is already holding a game piece.
   * 
   * @param depositPosition the row; bottom (hybrid), middle, top; to deposit the game piece in
   * @param stows stows after running
   * @param elevator the elevator subsystem
   * @param wrist the wrist subsystem
   * @param intake the intake subsystem
   * @param isCone a boolean supplier that returns true if the intake is holding a cone, false if cube
   */
  public AutoDeposit(Position depositPosition, GamePieceType gamePieceType, boolean stows, Elevator elevator, Wrist wrist, Intake intake) {
    addRequirements(elevator, wrist, intake);

    Command depositCommand = new DoNothing();


    if (gamePieceType == GamePieceType.CONE) {
      // Cone scoring
      if (depositPosition == Position.TOP) {
        depositCommand = new MoveElevator(elevator, ElevatorConstants.kAutoTopCone)
          .alongWith(new WaitCommand(0.6).andThen(new RotateWrist(wrist, WristConstants.kAutoTopCone)));
      } else if (depositPosition == Position.MIDDLE) {
        depositCommand = new MoveElevator(elevator, ElevatorConstants.kAutoMiddle)
          .alongWith(new WaitCommand(0.4).andThen(new RotateWrist(wrist, WristConstants.kAutoMiddle)));
      }
    } else {
      // Cube scoring
      if (depositPosition == Position.TOP || depositPosition == Position.MIDDLE) {
        depositCommand = new PositionIntake(elevator, wrist, GamePieceType.CUBE, depositPosition);
      } else {
        depositCommand = new PositionIntake(elevator, wrist, () -> false, Position.BOTTOM);
      }
    }

    addCommands(
      new ConditionalCommand(new DoNothing(), new CalibrateElevator(elevator), () -> elevator.isCalibrated()),
      depositCommand,
      new OuttakeGamePiece(intake, () -> gamePieceType),
      stows ? new Stow(elevator, wrist) : new DoNothing()
    );
  }
}
