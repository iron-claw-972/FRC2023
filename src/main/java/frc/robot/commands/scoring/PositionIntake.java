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

public class PositionIntake extends SequentialCommandGroup {
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
          new MoveElevator(elevator, ElevatorConstants.kShelfConeHeight).alongWith(new RotateWrist(wrist, WristConstants.kIntakeConePos)),
          new MoveElevator(elevator, ElevatorConstants.kShelfCubeHeight).alongWith(new RotateWrist(wrist, WristConstants.kIntakeCubePos)),
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

  public enum Position {
    TOP, MIDDLE, BOTTOM, SHELF, INTAKE, STOW
  }
}
