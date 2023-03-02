package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorMode;

public class MoveElevator extends SequentialCommandGroup {
  public MoveElevator(Elevator elevator, double desiredHeight) {
    addRequirements(elevator);
    addCommands(
      new InstantCommand(() -> elevator.setDesiredHeight(desiredHeight), elevator),
      new InstantCommand(() -> elevator.setMode(ElevatorMode.POSITION), elevator)
    );
  }
}
