package frc.robot.commands.scoring;

import java.util.Map;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.scoring.wrist.RotateWrist;
import frc.robot.commands.scoring.elevator.MoveElevator;
import frc.robot.constants.WristConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;
import frc.robot.util.GamePieceType;

public class PositionIntake extends SequentialCommandGroup {
  
  /**
   * Moves the elevator and the wrist to the position, factoring in game piece type.
   * Uses constants from ElevatorConstants.java and WristConstants.java
   * @param elevator the elevator subsystem
   * @param wrist the wrist subsystem
   * @param gamePieceType the game piece to score
   * @param position what position to move the elevator/wrist to
   */
  public PositionIntake(Elevator elevator, Wrist wrist, GamePieceType gamePieceType, Position position) {
    this(elevator, wrist, () -> gamePieceType == GamePieceType.CONE, position);
  }

  /**
   * Moves the elevator and the wrist to the position, factoring in game piece type.
   * Uses constants from ElevatorConstants.java and WristConstants.java
   * @param elevator the elevator subsystem
   * @param wrist the wrist subsystem
   * @param isConeSupplier a supplier outputting true if the game piece is a cone, false if it is a cube
   * @param position what position to move the elevator/wrist to
   */
  public PositionIntake(Elevator elevator, Wrist wrist, BooleanSupplier isConeSupplier, Position position) {
    addRequirements(elevator, wrist);
    addCommands(
      new SelectCommand(Map.ofEntries(
        Map.entry(Position.TOP, new ConditionalCommand(
          new MoveElevator(elevator, ElevatorConstants.kTopConeHeight).alongWith(new RotateWrist(wrist, WristConstants.kTopNodeConePos)),
          new MoveElevator(elevator, ElevatorConstants.kTopCubeHeight).alongWith(new RotateWrist(wrist, WristConstants.kTopNodeCubePos)),
          isConeSupplier
        )),
        Map.entry(Position.MIDDLE, new ConditionalCommand(
          new MoveElevator(elevator, ElevatorConstants.kMiddleConeHeight).alongWith(new RotateWrist(wrist, WristConstants.kMiddleNodeConePos)),
          new MoveElevator(elevator, ElevatorConstants.kMiddleCubeHeight).alongWith(new RotateWrist(wrist, WristConstants.kMiddleNodeCubePos)),
          isConeSupplier
        )),
        Map.entry(Position.BOTTOM, new ConditionalCommand(
          new MoveElevator(elevator, ElevatorConstants.kBottomConeHeight).alongWith(new RotateWrist(wrist, WristConstants.kBottomNodeConePos)),
          new MoveElevator(elevator, ElevatorConstants.kBottomCubeHeight).alongWith(new RotateWrist(wrist, WristConstants.kBottomNodeCubePos)),
          isConeSupplier
        )),
        Map.entry(Position.SHELF, new ConditionalCommand(
          new MoveElevator(elevator, ElevatorConstants.kShelfHeight).alongWith(new RotateWrist(wrist, WristConstants.kIntakeShelfPos)),
          new MoveElevator(elevator, ElevatorConstants.kShelfHeight).alongWith(new RotateWrist(wrist, WristConstants.kIntakeShelfPos)),
          isConeSupplier
        )),
        Map.entry(Position.INTAKE, new ConditionalCommand(
          new MoveElevator(elevator, ElevatorConstants.kIntakeConeHeight).alongWith(new RotateWrist(wrist, WristConstants.kIntakeConePos)),
          new MoveElevator(elevator, ElevatorConstants.kIntakeCubeHeight).alongWith(new RotateWrist(wrist, WristConstants.kIntakeCubePos)),
          isConeSupplier
        )),
        Map.entry(Position.STOW,
          new MoveElevator(elevator, ElevatorConstants.kStowHeight).alongWith(new RotateWrist(wrist, WristConstants.kStowPos))
        )
        
      ), () -> position)
    );
  }

  /**
   * A enum representing what positions the elevator and wrist can move to 
   */
  public enum Position {
    TOP, MIDDLE, BOTTOM, SHELF, INTAKE, STOW
  }
}
