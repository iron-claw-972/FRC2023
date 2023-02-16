package frc.robot.constants;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;



public class VisionConstants {
  //If the robot has 2 cameras
  public static final boolean k2Cameras=true;

  //The names of the cameras
  public static final String kCameraName1 = "Camera_1";
  public static final String kCameraName2 = "Camera_2";

  //Distance and rotation from the center of the robot to the cameras
  public static final Transform3d kRobotToCamera1 = new Transform3d(new Translation3d(Units.inchesToMeters(10), 0, Units.inchesToMeters(5)), new Rotation3d(0, 0, Math.PI));
  public static final Transform3d kRobotToCamera2 = new Transform3d(new Translation3d(Units.inchesToMeters(-10), 0, Units.inchesToMeters(5)), new Rotation3d(0, 0, 0));
  
  //Currently not used. Do not change.
  public static final double kCamera1Rotation=0;
  public static final double kCamera2Rotation=0;

  //Array to use if it can't find the April tag field layout
  public static final ArrayList<AprilTag> kTagPoses = new ArrayList<AprilTag>(List.of(
    new AprilTag(2, new Pose3d())
  ));
}