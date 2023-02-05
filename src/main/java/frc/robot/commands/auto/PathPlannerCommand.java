package frc.robot.commands.auto;

import java.util.ArrayList;
import java.util.Arrays;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.DoNothing;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.PathGroupLoader;



// Assuming this method is part of a drivetrain subsystem that provides the necessary methods
public class PathPlannerCommand extends SequentialCommandGroup{
    private Drivetrain m_drive;

    public PathPlannerCommand(String pathGroupName, int pathIndex){
        this(pathGroupName, pathIndex, Robot.drive);

    }
    
    public PathPlannerCommand(ArrayList<PathPoint> waypoints) {
        this(PathPlanner.generatePath(
          new PathConstraints(Constants.auto.kMaxAutoSpeed, Constants.auto.kMaxAutoAccel),
          waypoints.get(0),
          waypoints.get(1),
          (PathPoint[]) Arrays.copyOfRange(waypoints.toArray(), 2, waypoints.size())
        ));
      }
    
    public PathPlannerCommand(PathPlannerTrajectory path){
    this(new ArrayList<PathPlannerTrajectory>(Arrays.asList(path)), 0, Robot.drive, false);
    }

    public PathPlannerCommand(String pathGroupName, int pathIndex, Drivetrain drive){
        this(PathGroupLoader.getPathGroup(pathGroupName), pathIndex, drive, true); 
    }
    public PathPlannerCommand(ArrayList<PathPlannerTrajectory> pathGroup, int pathIndex, Drivetrain drive, boolean resetPose){
        m_drive = drive;
        addRequirements(m_drive);
        if (pathIndex < 0 || pathIndex > pathGroup.size() - 1){
            throw new IndexOutOfBoundsException("Path index out of range"); 
        } 
        PathPlannerTrajectory path = PathPlannerTrajectory.transformTrajectoryForAlliance(pathGroup.get(pathIndex),
            DriverStation.getAlliance());

        addCommands(
            (pathIndex == 0 && resetPose ? new InstantCommand(() -> m_drive.resetOdometry(path.getInitialHolonomicPose(), m_drive.getRotation2d())) : new DoNothing()),
            new PrintCommand("Number of paths: " + pathGroup.size()),
            new PPSwerveControllerCommand(
                path, 
                m_drive::getPose, // Pose supplier
                Robot.drive.getXController(), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                Robot.drive.getYController(), // Y controller (usually the same values as X controller)
                Robot.drive.getRotationController(), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                m_drive::setChassisSpeeds, // chassis speed consumer
                false,  // Do not transform path
                m_drive // Requires this drive subsystem
            )
        );
    }


    
}
