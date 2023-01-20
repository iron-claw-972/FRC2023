package frc.robot.util;

import java.io.File;
import java.util.ArrayList;
import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.constants.Constants;

public class PathGroupLoader{

  private static HashMap<String, ArrayList<PathPlannerTrajectory>> pathGroups = new HashMap<>();

  public static void loadPathGroups() {
    double totalTime = 0;
    File[] directoryListing = Filesystem.getDeployDirectory().toPath().resolve(Constants.auto.kTrajectoryDirectory).toFile().listFiles();
    if (directoryListing != null) {
      for (File file : directoryListing) {
        if (file.isFile() && file.getName().indexOf(".") != -1) {
          long startTime = System.nanoTime();
          String name = file.getName().substring(0, file.getName().indexOf("."));
          pathGroups.put(name, new ArrayList<>(PathPlanner.loadPathGroup(name, new PathConstraints(Constants.auto.kMaxAutoSpeed, Constants.auto.kMaxAutoAccel))));
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

  public static ArrayList<PathPlannerTrajectory> getPathGroup(String pathGroupName) {
    if (pathGroups.get(pathGroupName) == null) {
      System.out.println("Error retriving " + pathGroupName + " path!");
    }
    return pathGroups.get(pathGroupName);
  }


}