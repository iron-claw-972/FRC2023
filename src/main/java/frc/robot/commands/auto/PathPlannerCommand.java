package frc.robot.commands.auto;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.AutoConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.Conversions;
import frc.robot.util.PathGroupLoader;


public class PathPlannerCommand extends SequentialCommandGroup {
  
  public PathPlannerCommand(ArrayList<PathPoint> waypoints, Drivetrain drive) {
    this(waypoints, drive, true);
  }

  public PathPlannerCommand(ArrayList<PathPoint> waypoints, Drivetrain drive, boolean useAllianceColor) {
    this(new ArrayList<PathPlannerTrajectory>(Arrays.asList(PathPlanner.generatePath(
      new PathConstraints(AutoConstants.kMaxAutoSpeed, AutoConstants.kMaxAutoAccel),
      waypoints.get(0),
      waypoints.get(1)
    ))), 0, drive, false, useAllianceColor, true);
  }
  
  public PathPlannerCommand(String pathGroupName, int pathIndex, Drivetrain drive) {
    this(PathGroupLoader.getPathGroup(pathGroupName), pathIndex, drive, true, true, false); 
  }
  
  public PathPlannerCommand(String pathGroupName, int pathIndex, Drivetrain drive, boolean resetPose) {
    this(PathGroupLoader.getPathGroup(pathGroupName), pathIndex, drive, resetPose, true, false); 
  }

  public PathPlannerCommand(List<PathPlannerTrajectory> pathGroup, int pathIndex, Drivetrain drive, boolean resetPose){
    this(pathGroup, pathIndex, drive, resetPose, true, false);
  }

  public PathPlannerCommand(List<PathPlannerTrajectory> pathGroup, int pathIndex, Drivetrain drive, boolean resetPose, boolean useAllianceColor, boolean isPerpetual) {

    addRequirements(drive);
    if (pathIndex < 0 || pathIndex > pathGroup.size() - 1) {
      throw new IndexOutOfBoundsException("Path index out of range"); 
    }
    
    addCommands(
      new InstantCommand( () -> {
        PathPlannerTrajectory path = PathPlannerTrajectory.transformTrajectoryForAlliance(
          pathGroup.get(pathIndex), DriverStation.getAlliance());
        if (resetPose) {
          drive.resetOdometry(Conversions.absolutePoseToPathPlannerPose(path.getInitialHolonomicPose(), DriverStation.getAlliance()));
        }
      }),
      createSwerveControllerCommand(
        pathGroup.get(pathIndex), 
        useAllianceColor ? // Pose supplier
          () -> Conversions.absolutePoseToPathPlannerPose(drive.getPose(), DriverStation.getAlliance()) : 
          () -> drive.getPose(), 
        drive.getPathplannerXController(), // X controller can't normal PID as pathplanner has Feed Forward 
        drive.getPathplannerYController(), // Y controller can't normal PID as pathplanner has Feed Forward 
        drive.getPathplannerRotationController(), // Rotation controller can't normal PID as pathplanner has Feed Forward 
        (chassisSpeeds) -> { drive.setChassisSpeeds(chassisSpeeds, false); }, // chassis Speeds consumer
        useAllianceColor,  // use Alliance color
        drive, // Requires this drive subsystem
        isPerpetual
      )
    );
  }
  
  public static PPSwerveControllerCommand createSwerveControllerCommand(
      PathPlannerTrajectory trajectory, Supplier<Pose2d> poseSupplier, 
      PIDController xController, PIDController yController, PIDController rotationController, 
      Consumer<ChassisSpeeds> outputChassisSpeeds, boolean useAllianceColor, Drivetrain drive, boolean isPerpetual) {
    if (isPerpetual) {
      return new PPSwerveControllerCommandPerpetual(
        trajectory,
        poseSupplier,
        xController,
        yController,
        rotationController,
        outputChassisSpeeds,
        useAllianceColor,
        drive
      );
    }

    return new PPSwerveControllerCommand(
      trajectory,
      poseSupplier,
      xController,
      yController,
      rotationController,
      outputChassisSpeeds,
      useAllianceColor,
      drive
    );
  }
}
