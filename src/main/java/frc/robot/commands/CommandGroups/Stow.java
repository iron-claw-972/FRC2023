package frc.robot.commands.CommandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.ExtendToPosition;
import frc.robot.commands.elevator.MoveToExtension;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.FourBarArm;

public class Stow extends SequentialCommandGroup {
  public Stow(Elevator elevator, FourBarArm arm) {
    addRequirements(elevator, arm);
    addCommands(
      new SequentialCommandGroup(
        new ExtendToPosition(arm, ArmConstants.kInitialPosition),
        new MoveToExtension(elevator, ElevatorConstants.kMinExtension)

      )
    );
  }
}
