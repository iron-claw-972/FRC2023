package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

/**
 * Sets the robot's wheels to an X formation to prevent being pushed around by other bots.
 */
public class SetFormationX extends CommandBase {
  Drivetrain m_drive;

  /**
   * Creates a new command.
   * @param drive the drivetrain instance
   */
  public SetFormationX(Drivetrain drive) {
    this.m_drive=drive;
    addRequirements(drive);
  }

  public void execute() {
    m_drive.setModuleStates(new SwerveModuleState[] {
      new SwerveModuleState(0.01, new Rotation2d(Units.degreesToRadians(-45))),
      new SwerveModuleState(0.01, new Rotation2d(Units.degreesToRadians(45))),
      new SwerveModuleState(0.01, new Rotation2d(Units.degreesToRadians(-45))),
      new SwerveModuleState(0.01, new Rotation2d(Units.degreesToRadians(45)))
    });
  }
}