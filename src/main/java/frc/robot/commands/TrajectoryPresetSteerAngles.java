package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.swerve.DriveConstants;
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
      State sample = trajectory.sample(time);
      Pose2d nextPose = sample.poseMeters;
  
      double xVelocity = sample.velocityMetersPerSecond * nextPose.getRotation().getCos();
      double yVelocity = sample.velocityMetersPerSecond * nextPose.getRotation().getSin();
      double angularVelo = (nextPose.getRotation().getRadians() - initialPose.getRotation().getRadians()) / time;
  
      ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xVelocity, yVelocity, angularVelo, initialPose.getRotation());
  
      SwerveModuleState[] swerveModuleStates = DriveConstants.kKinematics.toSwerveModuleStates(chassisSpeeds);
      for (int i = 0; i < swerveModuleStates.length; i++){
        swerveModuleStates[i].speedMetersPerSecond = 0;
      }
      drive.setModuleStates(swerveModuleStates, true);
      drive.enableStateDeadband(true);
    }, 
    drive
    );
    
  }
}
