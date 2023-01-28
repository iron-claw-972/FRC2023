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
  public final boolean k2Cameras=true;
  public final String kCameraName1 = "Global_Shutter_Camera";
  public final String kCameraName2 = "Global_Shutter_Camera_2";
  public final Transform3d kCameraToRobot1 = new Transform3d(new Translation3d(Units.inchesToMeters(12.5), 0, Units.inchesToMeters(5)), new Rotation3d(0, 0, 0)); 
  public final Transform3d kCameraToRobot2 = new Transform3d(new Translation3d(Units.inchesToMeters(12.5), 0, Units.inchesToMeters(5)), new Rotation3d(0, 0, 0)); 
  public final ArrayList<AprilTag> kTagPoses = new ArrayList<AprilTag>(List.of(
    new AprilTag(2, new Pose3d())
  ));
}