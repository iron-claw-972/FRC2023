package frc.robot.util;

import java.io.File;
import java.io.IOException;
import java.util.HashMap;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.constants.Constants;

public class Functions {

  private static HashMap<String, Trajectory> trajectories = new HashMap<String, Trajectory>();

  /**
   * Deadbands an input to [-1, -deadband], [deadband, 1], rescaling inputs to be
   * linear from (deadband, 0) to (1,1)
   * 
   * @param input    The input value to rescale
   * @param deadband The deadband
   * @return the input rescaled and to fit [-1, -deadband], [deadband, 1]
   */
  public static double deadband(double input, double deadband) {
    if (Math.abs(input) <= deadband) {
        return 0;
    } else if (Math.abs(input) == 1) {
        return input;
    } else {
        return (1 / (1 - deadband) * (input + Math.signum(-input) * deadband));
    }
  }

  /**
   * an exponential function that maintains positive or negative 
  */ 
  public static double expoMS(double exponent, double base) {
    //weird stuff will happen if you don't put a number > 0
    double finVal = Math.pow(Math.abs(base),exponent);
    if (base < 0) {
      finVal *= -1;
    }
    return finVal;
  }

  public static void loadPaths() {
    double totalTime = 0;
    File[] directoryListing = Filesystem.getDeployDirectory().toPath().resolve(Constants.auto.kTrajectoryDirectory).toFile().listFiles();
    if (directoryListing != null) {
      for (File file : directoryListing) {
        if (file.isFile() && file.getName().indexOf(".") != -1) {
          long startTime = System.nanoTime();
          String name = file.getName().substring(0, file.getName().indexOf("."));
          trajectories.put(name, getTrajectoryFromJson(name));
          double time = (System.nanoTime() - startTime) / 1000000.0;
          totalTime += time;
          System.out.println("Processed file: " + file.getName() + ", took " + time + " milliseconds.");
        }
      }
    } else {
      DriverStation.reportWarning(
        "Issue with finding path files. Paths will not be loaded.",
        true
      );
    }
    System.out.println("File processing took a total of " + totalTime + " milliseconds");
  }

  public static Trajectory getTrajectory(String trajectoryName) {
    return trajectories.get(trajectoryName);
  }

  private static Trajectory getTrajectoryFromJson(String trajectoryName) {
    try {
      return TrajectoryUtil.fromPathweaverJson(
        Filesystem.getDeployDirectory().toPath().resolve(
          Constants.auto.kTrajectoryDirectory + trajectoryName + ".wpilib.json"
        )
      );
    } catch (IOException ex) {
      DriverStation.reportWarning(
        "Unable to open trajectory: " + trajectoryName + "\n" + "Will not do anything.",
        ex.getStackTrace()
      );
      return null;
    }
  }

  /**
   * Checks if a number is within (inclusive) a certain deviation of the target
   * @param actual the input number to check
   * @param target the target value
   * @param deviation the amount they can be within each other
   * @return if the actual is within devation distance from the target value
   */
  public static boolean withinDeviation(double actual, double target, double deviation) {
    return Math.abs(target-actual) <= deviation;
  }
}