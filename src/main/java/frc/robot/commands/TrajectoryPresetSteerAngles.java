package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

/**
 * Sets all module angles to a given trajectory's initial angle.
 */
public class TrajectoryPresetSteerAngles extends CommandBase {

  private final Drivetrain m_drive;
  private final Trajectory m_trajectory;
  
  public TrajectoryPresetSteerAngles(Drivetrain drive, Trajectory trajectory) {
    m_drive = drive;
    m_trajectory = trajectory;
    
    addRequirements(drive);
  }
  
  @Override
  public void initialize() {

    // 0.01 is the time between trajectory samples, in seconds
    // Can be replaced for any small number, but it should be the same as the time between all uses
    double time = 0.01;

    m_drive.enableStateDeadband(false);

    Pose2d initialPose = m_trajectory.getInitialPose();
    Pose2d nextPose = m_trajectory.sample(time).poseMeters;

    double xVelo = (nextPose.getX() - initialPose.getX()) / time;
    double yVelo = (nextPose.getY() - initialPose.getY()) / time;
    double angularVelo = (nextPose.getRotation().getRadians() - initialPose.getRotation().getRadians()) / time;

    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xVelo, yVelo, angularVelo, initialPose.getRotation());

    m_drive.setModuleStates(new SwerveModuleState[]{
      new SwerveModuleState(0, Rotation2d.fromRadians(chassisSpeeds.omegaRadiansPerSecond)),
      new SwerveModuleState(0, Rotation2d.fromRadians(chassisSpeeds.omegaRadiansPerSecond)),
      new SwerveModuleState(0, Rotation2d.fromRadians(chassisSpeeds.omegaRadiansPerSecond)),
      new SwerveModuleState(0, Rotation2d.fromRadians(chassisSpeeds.omegaRadiansPerSecond))
    });
  }

  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.enableStateDeadband(true);
  }
    
}
