package frc.robot.commands.CommandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.FourBarArm;
import frc.robot.subsystems.Intake;

public class IntakeFromGround extends SequentialCommandGroup {
  public IntakeFromGround(Elevator elevator, FourBarArm arm, Intake intake) {
    addRequirements(elevator, arm, intake);
    addCommands(
      new SequentialCommandGroup(
      //TODO: Put in elevator extension for double substation                  new MoveToExtension(elevator, ElevatorConstants.),
      //TODO: Intake command here
      new Stow(elevator, arm)
      )
    );
  }
}
