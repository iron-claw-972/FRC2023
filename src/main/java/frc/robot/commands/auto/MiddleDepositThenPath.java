package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
import frc.robot.util.PathGroupLoader;

public class MiddleDepositThenPath extends SequentialCommandGroup {
  public MiddleDepositThenPath(String pathName, Drivetrain drive, Elevator elevator, FourBarArm arm, Intake intake) {
    addRequirements(drive, elevator, arm, intake);
    addCommands(
      new InstantCommand(() -> drive.setPigeonYaw(PathGroupLoader.getPathGroup(pathName).get(0))),
      new PositionIntake(elevator, arm, () -> false, Position.MIDDLE).withTimeout(3),
      new Dunk(arm, intake).withTimeout(1.5),
      new ParallelCommandGroup(
        new PathPlannerCommand(pathName, 0, drive, true),
        new WaitCommand(1).andThen(new Stow(intake, elevator, arm))
      )
    );
  }
}
