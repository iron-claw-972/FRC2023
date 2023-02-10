package frc.robot.constants;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;



public class VisionConstants {
  //If the robot has 2 cameras
  public final boolean k2Cameras=true;

  //The names of the cameras
  public final String kCameraName1 = "Camera_1";
  public final String kCameraName2 = "Camera_2";

  //Distance and rotation from the center of the robot to the cameras
  public final Transform3d kRobotToCamera1 = new Transform3d(new Translation3d(Units.inchesToMeters(10), 0, Units.inchesToMeters(5)), new Rotation3d(0, 0, Math.PI));
  public final Transform3d kRobotToCamera2 = new Transform3d(new Translation3d(Units.inchesToMeters(-10), 0, Units.inchesToMeters(5)), new Rotation3d(0, 0, 0));
  
  //Currently not used. Do not change.
  public final double kCamera1Rotation=0;
  public final double kCamera2Rotation=0;

  //Array to use if it can't find the April tag field layout
  public final ArrayList<AprilTag> kTagPoses = new ArrayList<AprilTag>(List.of(
    new AprilTag(2, new Pose3d())
  ));
}