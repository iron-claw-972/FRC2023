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

public class PositionRollerIntake extends SequentialCommandGroup {
  public PositionRollerIntake(Elevator elevator, Wrist wrist, BooleanSupplier isConeSupplier, RollerPosition position) {
    addRequirements(elevator, wrist);
    addCommands(
      new SelectCommand(Map.ofEntries(
        Map.entry(RollerPosition.TOP, new ConditionalCommand(
          new MoveElevator(elevator, ElevatorConstants.kTopConeHeight).alongWith(new RotateWrist(wrist, WristConstants.kTopNodePos)),
          new MoveElevator(elevator, ElevatorConstants.kTopCubeHeight).alongWith(new RotateWrist(wrist, WristConstants.kTopNodePos)),
          isConeSupplier
        )),
        Map.entry(RollerPosition.MIDDLE, new ConditionalCommand(
          new MoveElevator(elevator, ElevatorConstants.kMiddleConeHeight).alongWith(new RotateWrist(wrist, WristConstants.kMiddleNodePos)),
          new MoveElevator(elevator, ElevatorConstants.kMiddleCubeHeight).alongWith(new RotateWrist(wrist, WristConstants.kMiddleNodePos)),
          isConeSupplier
        )),
        Map.entry(RollerPosition.BOTTOM, new ConditionalCommand(
          new MoveElevator(elevator, ElevatorConstants.kBottomConeHeight).alongWith(new RotateWrist(wrist, WristConstants.kBottomNodePos)),
          new MoveElevator(elevator, ElevatorConstants.kBottomCubeHeight).alongWith(new RotateWrist(wrist, WristConstants.kBottomNodePos)),
          isConeSupplier
        )),
        Map.entry(RollerPosition.SHELF, new ConditionalCommand(
          new MoveElevator(elevator, ElevatorConstants.kShelfConeHeight).alongWith(new RotateWrist(wrist, WristConstants.kShelfPos)),
          new MoveElevator(elevator, ElevatorConstants.kShelfCubeHeight).alongWith(new RotateWrist(wrist, WristConstants.kShelfPos)),
          isConeSupplier
        )),
        Map.entry(RollerPosition.INTAKE, new ConditionalCommand(
          new MoveElevator(elevator, ElevatorConstants.kIntakeConeHeight).alongWith(new RotateWrist(wrist, WristConstants.kIntakeConePos)),
          new MoveElevator(elevator, ElevatorConstants.kIntakeCubeHeight).alongWith(new RotateWrist(wrist, WristConstants.kIntakeCubePos)),
          isConeSupplier
        ))
        
      ), () -> position)
    );
  }

  public enum RollerPosition {
    TOP, MIDDLE, BOTTOM, SHELF, INTAKE
  }
}
