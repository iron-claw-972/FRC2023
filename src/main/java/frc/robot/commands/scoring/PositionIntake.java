package frc.robot.commands.scoring;

import java.util.Map;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.scoring.arm.ExtendArm;
import frc.robot.commands.scoring.elevator.MoveElevator;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.FourBarArm;

public class PositionIntake extends SequentialCommandGroup {
  public PositionIntake(Elevator elevator, FourBarArm arm, BooleanSupplier isConeColorSupplier, Position position) {
    addRequirements(elevator, arm);
    addCommands(
      new SelectCommand(Map.ofEntries(
        Map.entry(Position.TOP, new ConditionalCommand(
          new MoveElevator(elevator, ElevatorConstants.kTopConeHeight).alongWith(new ExtendArm(arm, ArmConstants.kTopNodePos)),
          new MoveElevator(elevator, ElevatorConstants.kTopCubeHeight).alongWith(new ExtendArm(arm, ArmConstants.kTopNodePos)),
          isConeColorSupplier
        )),
        Map.entry(Position.MIDDLE, new ConditionalCommand(
          new MoveElevator(elevator, ElevatorConstants.kMiddleConeHeight).alongWith(new ExtendArm(arm, ArmConstants.kMiddleNodePos)),
          new MoveElevator(elevator, ElevatorConstants.kMiddleCubeHeight).alongWith(new ExtendArm(arm, ArmConstants.kMiddleNodePos)),
          isConeColorSupplier
        )),
        Map.entry(Position.BOTTOM, new ConditionalCommand(
          new MoveElevator(elevator, ElevatorConstants.kBottomConeHeight).alongWith(new ExtendArm(arm, ArmConstants.kBottomNodePos)),
          new MoveElevator(elevator, ElevatorConstants.kBottomCubeHeight).alongWith(new ExtendArm(arm, ArmConstants.kBottomNodePos)),
          isConeColorSupplier
        )),
        Map.entry(Position.SHELF, new ConditionalCommand(
          new MoveElevator(elevator, ElevatorConstants.kShelfConeHeight).alongWith(new ExtendArm(arm, ArmConstants.kShelfPos)),
          new MoveElevator(elevator, ElevatorConstants.kShelfCubeHeight).alongWith(new ExtendArm(arm, ArmConstants.kShelfPos)),
          isConeColorSupplier
        )),
        Map.entry(Position.INTAKE, new ConditionalCommand(
          new MoveElevator(elevator, ElevatorConstants.kIntakeConeHeight).alongWith(new ExtendArm(arm, ArmConstants.kIntakePos)),
          new MoveElevator(elevator, ElevatorConstants.kIntakeCubeHeight).alongWith(new ExtendArm(arm, ArmConstants.kIntakePos)),
          isConeColorSupplier
        )),
        Map.entry(Position.STOW, 
          new MoveElevator(elevator, ElevatorConstants.kStowHeight).alongWith(new ExtendArm(arm, ArmConstants.kStowPos))
        )
      ), () -> position)
    );
  }

  public enum Position {
    TOP, MIDDLE, BOTTOM, SHELF, INTAKE, STOW
  }
}
