package frc.robot.commands.auto;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.AutoConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.Conversions;
import frc.robot.util.PathGroupLoader;


public class PathPlannerCommand extends SequentialCommandGroup {
  
  Alliance alliance;
  boolean m_continuous;
  
  public PathPlannerCommand(ArrayList<PathPoint> waypoints, Drivetrain drive) {
    this(new ArrayList<PathPlannerTrajectory>(Arrays.asList(PathPlanner.generatePath(
      new PathConstraints(AutoConstants.kMaxAutoSpeed, AutoConstants.kMaxAutoAccel),
      waypoints.get(0),
      waypoints.get(1),
      (PathPoint[]) Arrays.copyOfRange(waypoints.toArray(), 2, waypoints.size())
    ))), 0, drive, true);
  }
  
  public PathPlannerCommand(String pathGroupName, int pathIndex, Drivetrain drive) {
    this(PathGroupLoader.getPathGroup(pathGroupName), pathIndex, drive, true); 
  }
  
  public PathPlannerCommand(String pathGroupName, int pathIndex, Drivetrain drive, boolean resetPose) {
    this(PathGroupLoader.getPathGroup(pathGroupName), pathIndex, drive, resetPose); 
  }
  
  public PathPlannerCommand(List<PathPlannerTrajectory> pathGroup, int pathIndex, Drivetrain drive, boolean resetPose) {
    this(pathGroup, pathIndex, drive, resetPose, false);
  }
  
  public PathPlannerCommand(List<PathPlannerTrajectory> pathGroup, int pathIndex, Drivetrain drive, boolean resetPose, boolean continuos) {
    m_continuous = continuos;
    addRequirements(drive);
    if (pathIndex < 0 || pathIndex > pathGroup.size() - 1) {
      throw new IndexOutOfBoundsException("Path index out of range"); 
    }
    
    addCommands(new InstantCommand( () -> {
      PathPlannerTrajectory path = PathPlannerTrajectory.transformTrajectoryForAlliance(
        pathGroup.get(pathIndex), DriverStation.getAlliance());
      if (resetPose) {
        drive.resetOdometry(Conversions.absolutePoseToPathPlannerPose(path.getInitialHolonomicPose(), DriverStation.getAlliance()));
      }
    }), 
    (m_continuous ? 
      new PPSwerveControllerCommandContinuous(
        pathGroup.get(pathIndex), 
        () -> Conversions.absolutePoseToPathPlannerPose(
        drive.getPose(), DriverStation.getAlliance()), // Pose supplier
        drive.getPathplannerXController(), // X controller can't normal PID as pathplanner has Feed Forward 
        drive.getPathplannerYController(), // Y controller can't normal PID as pathplanner has Feed Forward 
        drive.getPathplannerRotationController(), // Rotation controller can't normal PID as pathplanner has Feed Forward 
        (chassisSpeeds) -> { drive.setChassisSpeeds(chassisSpeeds, false); }, // chassis Speeds consumer
        true,  // use Alliance color
        drive // Requires this drive subsystem
      ) :
      new PPSwerveControllerCommand(
        pathGroup.get(pathIndex), 
        () -> Conversions.absolutePoseToPathPlannerPose(
        drive.getPose(), DriverStation.getAlliance()), // Pose supplier
        drive.getPathplannerXController(), // X controller can't normal PID as pathplanner has Feed Forward 
        drive.getPathplannerYController(), // Y controller can't normal PID as pathplanner has Feed Forward 
        drive.getPathplannerRotationController(), // Rotation controller can't normal PID as pathplanner has Feed Forward 
        (chassisSpeeds) -> { drive.setChassisSpeeds(chassisSpeeds, false); }, // chassis Speeds consumer
        true,  // use Alliance color
        drive // Requires this drive subsystem
      )
    ));
  }
}
