package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.constants.AutoConstants;

import java.io.File;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

/**
 * Utility class for loading paths using pathplanner.
 */
public class PathGroupLoader {

    // private static final HashMap<String, List<PathPlannerTrajectory>> pathGroups = new HashMap<>();
    private static final HashMap<String, List<PathPlannerPath>> pathGroups = new HashMap<>();


    /**
     * Loads all the paths in the trajectory directory (specified in the constants).
     * These paths are loaded and stored so that they do not take time while the robot is running
     * and can be accessed with {@link #getPathGroup(String)}
     */
    public static void loadPathGroups() {
        double totalTime = 0;
        File[] directoryListing = Filesystem.getDeployDirectory().toPath().resolve(AutoConstants.TRAJECTORY_DIRECTORY).toFile().listFiles();
        if (directoryListing != null) {
            for (File file : directoryListing) {
                if (file.isFile() && file.getName().contains(".")) {
                    long startTime = System.nanoTime();
                    String name = file.getName().substring(0, file.getName().indexOf("."));
                    pathGroups.put(name, PathPlannerAuto.getPathGroupFromAutoFile(name));
                    // pathGroups.put(name, PathPlanner.loadPathGroup(name, new PathConstraints(AutoConstants.MAX_AUTO_SPEED, AutoConstants.MAX_AUTO_ACCEL)));
                    double time = (System.nanoTime() - startTime) / 1000000.0;
                    totalTime += time;
                    System.out.println("Processed file: " + file.getName() + ", took " + time + " milliseconds.");
                }
            }
        } else {
            System.out.println("Error processing file");
            DriverStation.reportWarning(
                    "Issue with finding path files. Paths will not be loaded.",
                    true
                                       );
        }
        System.out.println("File processing took a total of " + totalTime + " milliseconds");
    }

    /**
     * Gets a path that has already been loaded with {@link #loadPathGroups()}. The path group is a list
     * of trajectories that path planner can run.
     *
     * @param pathGroupName the name of the file, without any extensions. This should be the same exact name that is displayed in pathplanner
     * @return a list of trajectories that path planner can run.
     */
    public static List<PathPlannerPath> getPathGroup(String pathGroupName) {
        if (pathGroups.get(pathGroupName) == null) {
            System.out.println("Error retrieving " + pathGroupName + " path!");
        }
        return pathGroups.get(pathGroupName);
    }


}