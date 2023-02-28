package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Drivetrain;

/**
 * Sets all module angles to a given trajectory's initial angle.
 */
public class TrajectoryPresetSteerAngles extends InstantCommand {
  /*
   * make sure to add wait command after called to give time to correct
   */
  public TrajectoryPresetSteerAngles(Drivetrain drive, Trajectory trajectory) {
    super(
      ()->  {

      // 0.01 is the time between trajectory samples, in seconds
      // Can be replaced for any small number, but it should be the same as the time between all uses
      double time = 0.01;
  
      drive.enableStateDeadband(false);
  
      Pose2d initialPose = trajectory.getInitialPose();
      Pose2d nextPose = trajectory.sample(time).poseMeters;
  
      double xVelo = (nextPose.getX() - initialPose.getX()) / time;
      double yVelo = (nextPose.getY() - initialPose.getY()) / time;
      double angularVelo = (nextPose.getRotation().getRadians() - initialPose.getRotation().getRadians()) / time;
  
      ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xVelo, yVelo, angularVelo, initialPose.getRotation());
  
      drive.setModuleStates(new SwerveModuleState[]{
        new SwerveModuleState(0, Rotation2d.fromRadians(chassisSpeeds.omegaRadiansPerSecond)),
        new SwerveModuleState(0, Rotation2d.fromRadians(chassisSpeeds.omegaRadiansPerSecond)),
        new SwerveModuleState(0, Rotation2d.fromRadians(chassisSpeeds.omegaRadiansPerSecond)),
        new SwerveModuleState(0, Rotation2d.fromRadians(chassisSpeeds.omegaRadiansPerSecond))
      });
      drive.enableStateDeadband(true);
    }, 
    drive
    );
    
  }
}
