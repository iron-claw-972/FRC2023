package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.scoring.Stow;
import frc.robot.commands.scoring.Dunk;
import frc.robot.commands.scoring.PositionIntake;
import frc.robot.commands.scoring.PositionIntake.Position;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.FourBarArm;
import frc.robot.subsystems.Intake;

public class MiddleDepositThenPath extends SequentialCommandGroup {
  public MiddleDepositThenPath(String pathName, Drivetrain drive, Elevator elevator, FourBarArm arm, Intake intake) {
    addRequirements(drive, elevator, arm, intake);
    addCommands(
      new PositionIntake(elevator, arm, () -> false, Position.MIDDLE).withTimeout(3),
      new Dunk(arm, intake).withTimeout(1.5),
      new ParallelCommandGroup(
        new PathPlannerCommand(pathName, 0, drive),
        new WaitCommand(1.5).andThen(new Stow(intake, elevator, arm))
      )
    );
  }
}
