package frc.robot.commands.auto;

import java.util.function.Consumer;
import java.util.function.Supplier;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class PPSwerveControllerCommandPerpetual extends PPSwerveControllerCommand{

  public PPSwerveControllerCommandPerpetual(
      PathPlannerTrajectory trajectory,
      Supplier<Pose2d> poseSupplier,
      PIDController xController,
      PIDController yController,
      PIDController rotationController,
      Consumer<ChassisSpeeds> outputChassisSpeeds,
      boolean useAllianceColor,
      Subsystem... requirements) {
    super(
      trajectory,
      poseSupplier,
      xController,
      yController,
      rotationController,
      outputChassisSpeeds,
      useAllianceColor,
      requirements);
  }

  public PPSwerveControllerCommandPerpetual(
      PathPlannerTrajectory trajectory,
      Supplier<Pose2d> poseSupplier,
      PIDController xController,
      PIDController yController,
      PIDController rotationController,
      Consumer<ChassisSpeeds> outputChassisSpeeds,
      Subsystem... requirements) {
    super(
      trajectory,
      poseSupplier,
      xController,
      yController,
      rotationController,
      outputChassisSpeeds,
      requirements);
  }

  public PPSwerveControllerCommandPerpetual(
      PathPlannerTrajectory trajectory,
      Supplier<Pose2d> poseSupplier,
      SwerveDriveKinematics kinematics,
      PIDController xController,
      PIDController yController,
      PIDController rotationController,
      Consumer<SwerveModuleState[]> outputModuleStates,
      boolean useAllianceColor,
      Subsystem... requirements) {
    super(
      trajectory,
      poseSupplier,
      kinematics,
      xController,
      yController,
      rotationController,
      outputModuleStates,
      useAllianceColor,
      requirements);
  }

  public PPSwerveControllerCommandPerpetual(
      PathPlannerTrajectory trajectory,
      Supplier<Pose2d> poseSupplier,
      SwerveDriveKinematics kinematics,
      PIDController xController,
      PIDController yController,
      PIDController rotationController,
      Consumer<SwerveModuleState[]> outputModuleStates,
      Subsystem... requirements) {
    super(
      trajectory,
      poseSupplier,
      kinematics,
      xController,
      yController,
      rotationController,
      outputModuleStates, 
      requirements);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
