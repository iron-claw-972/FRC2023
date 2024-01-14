package frc.robot.commands.auto;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.swerve.DriveConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.Conversions;
import frc.robot.util.PathGroupLoader;


public class PathPlannerCommand extends SequentialCommandGroup {
  
  public PathPlannerCommand(ArrayList<PathPoint> waypoints, Drivetrain drive) {
    this(waypoints, drive, true);
  }

  public PathPlannerCommand(ArrayList<PathPoint> waypoints, Drivetrain drive, boolean useAllianceColor) {
    this(new ArrayList<PathPlannerPath>(Arrays.asList(new PathPlannerPath(
      List.of(
        waypoints.get(0).position,
        waypoints.get(1).position),
      new PathConstraints(AutoConstants.kMaxAutoSpeed, AutoConstants.kMaxAutoAccel, DriveConstants.kMaxAngularSpeed, DriveConstants.kMaxAngularAccel),
      new GoalEndState(0, waypoints.get(1).rotationTarget.getTarget())
    ))), 0, drive, false, useAllianceColor, true);
  }

  public PathPlannerCommand(ArrayList<PathPoint> waypoints, Drivetrain drive, boolean useAllianceColor, double maxSpeed, double maxAccel) {
    this(new ArrayList<PathPlannerPath>(Arrays.asList(new PathPlannerPath(
      List.of(
        waypoints.get(0).position,
        waypoints.get(1).position),
      new PathConstraints(maxSpeed, maxAccel, DriveConstants.kMaxAngularSpeed, DriveConstants.kMaxAngularAccel),
      new GoalEndState(0, waypoints.get(1).rotationTarget.getTarget())
    ))), 0, drive, false, useAllianceColor, true);
  }
  
  public PathPlannerCommand(String pathGroupName, int pathIndex, Drivetrain drive) {
    this(PathGroupLoader.getPathGroup(pathGroupName), pathIndex, drive, true, true, false); 
  }
  
  public PathPlannerCommand(String pathGroupName, int pathIndex, Drivetrain drive, boolean resetPose) {
    this(PathGroupLoader.getPathGroup(pathGroupName), pathIndex, drive, resetPose, true, false); 
  }

  public PathPlannerCommand(List<PathPlannerPath> pathGroup, int pathIndex, Drivetrain drive, boolean resetPose){
    this(pathGroup, pathIndex, drive, resetPose, true, false);
  }

  public PathPlannerCommand(List<PathPlannerPath> pathGroup, int pathIndex, Drivetrain drive, boolean resetPose, boolean useAllianceColor, boolean isPerpetual) {

    addRequirements(drive);
    if (pathIndex < 0 || pathIndex > pathGroup.size() - 1) {
      throw new IndexOutOfBoundsException("Path index out of range"); 
    }
    
    addCommands(
      new InstantCommand( () -> {
        PathPlannerPath path = pathGroup.get(pathIndex);
        if(DriverStation.getAlliance()==Alliance.Red){
          path.flipPath();
        }
        if (resetPose) {
          drive.resetOdometry(path.getPreviewStartingHolonomicPose());
        }
        //TODO: This might be unnecessary, maybe delete it
        if(DriverStation.getAlliance()==Alliance.Red){
          path.flipPath();
        }
      }),
      createSwerveControllerCommand(
        pathGroup.get(pathIndex), 
        useAllianceColor ? // Pose supplier
          () -> Conversions.absolutePoseToPathPlannerPose(drive.getPose(), DriverStation.getAlliance()) : 
          () -> drive.getPose(), 
        ()-> drive.getChassisSpeeds(),
        (chassisSpeeds) -> { drive.setChassisSpeeds(chassisSpeeds, false); }, // chassis Speeds consumer
        AutoConstants.kMaxAutoSpeed,
        Math.sqrt(2)*DriveConstants.kTrackWidth/2,
        useAllianceColor,  // use Alliance color
        drive, // Requires this drive subsystem
        isPerpetual
      )
    );
  }
  
  public static FollowPathHolonomic createSwerveControllerCommand(
      PathPlannerPath path, Supplier<Pose2d> poseSupplier, 
      Supplier<ChassisSpeeds> speedSupplier,
      Consumer<ChassisSpeeds> outputChassisSpeeds, 
      double maxModuleSpeed,
      double driveBaseReadius,
      boolean useAllianceColor, Drivetrain drive, boolean isPerpetual) {
    if (isPerpetual) {
      return new PPSwerveControllerCommandPerpetual(
        path,
        poseSupplier,
        speedSupplier,
        outputChassisSpeeds,
        new HolonomicPathFollowerConfig(maxModuleSpeed, driveBaseReadius, new ReplanningConfig(false, false)),
        useAllianceColor,
        drive
      );
    }

    return new FollowPathHolonomic(
        path,
        poseSupplier,
        speedSupplier,
        outputChassisSpeeds,
        new HolonomicPathFollowerConfig(maxModuleSpeed, driveBaseReadius, new ReplanningConfig(false, false)),
        ()-> useAllianceColor,
        drive
    );
  }
}
