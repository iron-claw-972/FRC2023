package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;

/**
 * Sets the robot's wheels to an X formation to prevent being pushed around by other bots.
 */
public class SetFormationX extends SequentialCommandGroup {
  public SetFormationX(Drivetrain drive) {
    addRequirements(drive);
    addCommands(
      new InstantCommand(() -> drive.enableStateDeadband(false), drive),
      new InstantCommand(() -> drive.setModuleStates(new SwerveModuleState[] {
        new SwerveModuleState(0, new Rotation2d(Units.degreesToRadians(45))),
        new SwerveModuleState(0, new Rotation2d(Units.degreesToRadians(-45))),
        new SwerveModuleState(0, new Rotation2d(Units.degreesToRadians(-45))),
        new SwerveModuleState(0, new Rotation2d(Units.degreesToRadians(45)))
      }), drive),
      new InstantCommand(() -> drive.enableStateDeadband(true), drive)
    );
  }
}