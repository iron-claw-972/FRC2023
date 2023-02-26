package frc.robot.commands;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.AutoConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.PathGroupLoader;

public class PathPlannerCommand extends SequentialCommandGroup {
  public PathPlannerCommand(ArrayList<PathPoint> waypoints, Drivetrain drive) {
    this(PathPlanner.generatePath(new PathConstraints(AutoConstants.kMaxAutoSpeed, AutoConstants.kMaxAutoAccel), waypoints), drive);
  }
  
  public PathPlannerCommand(PathPlannerTrajectory path, Drivetrain drive){
    this(new ArrayList<>(Arrays.asList(path)), 0, drive, false);
  }
  
  public PathPlannerCommand(String pathGroupName, int pathIndex, Drivetrain drive){
    this(PathGroupLoader.getPathGroup(pathGroupName), pathIndex, drive, true); 
  }
  public PathPlannerCommand(List<PathPlannerTrajectory> pathGroup, int pathIndex, Drivetrain drive, boolean resetPose){
    addRequirements(drive);
    if (pathIndex < 0 || pathIndex > pathGroup.size() - 1){
      throw new IndexOutOfBoundsException("Path index out of range"); 
    } 
    PathPlannerTrajectory path = pathGroup.get(pathIndex);
    
    addCommands(
      (pathIndex == 0 && resetPose ? new InstantCommand(() -> drive.resetOdometry(path.getInitialHolonomicPose()), drive) : new DoNothing()),
      new PrintCommand("Number of paths: " + pathGroup.size()),
      new PPSwerveControllerCommand(
        path,
        drive::getPose, // Pose supplier
        drive.getXController(), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
        drive.getYController(), // Y controller (usually the same values as X controller)
        drive.getRotationController(), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
        drive::setChassisSpeeds, // ChassisSpeeds consumer
        true,
        drive // Requires this drive subsystem
      )
    );
  } 
}