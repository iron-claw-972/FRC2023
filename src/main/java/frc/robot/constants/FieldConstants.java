package frc.robot.constants;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

public class FieldConstants {
  public static final double kFieldLength = Units.inchesToMeters(54*12 + 3.25); // meters
  public static final double kFieldWidth = Units.inchesToMeters(26*12 + 3.5); // meters

  // The distance from the center of the field to any one of the staged game pieces in the x direction.
  // This value was obtained using the CAD of the field.
  public static final double kCenterToStagedPieceX = Units.inchesToMeters(47.361); // meters

  // The distance from the apriltag to the edge of the grid
  public static final double kAprilTagOffset = Units.inchesToMeters(14.06 - 0.26); // meters

  // The distance from center of apriltag to the cone node
  public static final double kConeNodeOffset = Units.inchesToMeters(22); // meters

  // Array to use if it can't find the April tag field layout
  public static final ArrayList<AprilTag> kAprilTags = new ArrayList<AprilTag>(List.of(
    new AprilTag(1, new Pose3d(Units.inchesToMeters(610.77), Units.inchesToMeters( 42.19), Units.inchesToMeters(18.22), new Rotation3d(0.0, 0.0, Math.PI))),
    new AprilTag(2, new Pose3d(Units.inchesToMeters(610.77), Units.inchesToMeters(108.19), Units.inchesToMeters(18.22), new Rotation3d(0.0, 0.0, Math.PI))),
    new AprilTag(3, new Pose3d(Units.inchesToMeters(610.77), Units.inchesToMeters(174.19), Units.inchesToMeters(18.22), new Rotation3d(0.0, 0.0, Math.PI))),
    new AprilTag(4, new Pose3d(Units.inchesToMeters(636.96), Units.inchesToMeters(265.74), Units.inchesToMeters(27.38), new Rotation3d(0.0, 0.0, Math.PI))),
    new AprilTag(5, new Pose3d(Units.inchesToMeters( 14.25), Units.inchesToMeters(265.74), Units.inchesToMeters(27.38), new Rotation3d(0.0, 0.0, 0.0))),
    new AprilTag(6, new Pose3d(Units.inchesToMeters( 40.45), Units.inchesToMeters(174.19), Units.inchesToMeters(18.22), new Rotation3d(0.0, 0.0, 0.0))),
    new AprilTag(7, new Pose3d(Units.inchesToMeters( 40.45), Units.inchesToMeters(108.19), Units.inchesToMeters(18.22), new Rotation3d(0.0, 0.0, 0.0))),
    new AprilTag(8, new Pose3d(Units.inchesToMeters( 40.45), Units.inchesToMeters( 42.19), Units.inchesToMeters(18.22), new Rotation3d(0.0, 0.0, 0.0)))
  ));

  // Where the robot needs to go to intake from the shelf
  public static double kBlueShelfX = kAprilTags.get(3).pose.getX();
  public static double kRedShelfX = kAprilTags.get(4).pose.getX();
  // 15 from april tag to edge of portal (where cones are accessible). That area is 34.21 inches wide, just aim for the center
  public static double kShelfY = kAprilTags.get(3).pose.getY() - Units.inchesToMeters(15 + (34.21 / 2));
}
