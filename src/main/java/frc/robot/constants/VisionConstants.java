package frc.robot.constants;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.Robot;
import frc.robot.Robot.RobotId;



public class VisionConstants {
  public static ArrayList<Pair<String, Transform3d>> kCameras = new ArrayList<Pair<String, Transform3d>>(List.of(
  ));

  //How far from the grid the robot should go to score in inches
  public static double kgridDistance = 24;


  public static void update() {
    if (Robot.kRobotId == RobotId.SwerveTest) {
      kCameras = new ArrayList<Pair<String, Transform3d>>(List.of(
        new Pair<String, Transform3d>(
          "Camera_2",
          new Transform3d(
            new Translation3d(Units.inchesToMeters(15), Units.inchesToMeters(-3.3125), Units.inchesToMeters(10)),
            new Rotation3d(0, 0, Math.PI)
          )
        )
      ));
    }
  }

  // How much to trust vision measurements normally
  public static final Matrix<N3, N1> kBaseVisionPoseStdDevs = new MatBuilder<>(Nat.N3(), Nat.N1()).fill(
    0.9, // x in meters (default=0.9)
    0.9, // y in meters (default=0.9)
    0.9 // heading in radians (default=0.9)
  );

  // Increasing this makes pose estimation trust vision measurements less as distance from Apriltags increases
  // This is how much is added to std dev for vision when closest visible Apriltag is 1 meter away
  public static final double kVisionPoseStdDevFactor = 0.1;

  public static final double kFieldLength = Units.inchesToMeters(54*12 + 3.25);
  public static final double kFieldWidth = Units.inchesToMeters(26*12 + 3.5);

  //Array to use if it can't find the April tag field layout
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
}