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

/**
 * Container class for vision constants.
 */
public class VisionConstants {


  public static ArrayList<Pair<String, Transform3d>> kCameras = new ArrayList<>();

  /**
   * Update the camera list based on the robot id detected from rio preferences.
   */
  // TODO: Maybe we should move this to the if/else block in RobotContainer.java, where subsystems are initialized/not initialized based on robotid
  public static void update() {
    if (Robot.kRobotId == RobotId.SwerveTest) {
      kCameras = new ArrayList<Pair<String, Transform3d>>(List.of(
        new Pair<String, Transform3d>(
          "Camera_2",
          new Transform3d(
            new Translation3d(-Units.inchesToMeters(4.75), Units.inchesToMeters(10.375), Units.inchesToMeters(10)),
            new Rotation3d(0, 0, 0)
          )
        )
      ));
    } else {
      kCameras = new ArrayList<Pair<String, Transform3d>>(List.of(
        new Pair<String, Transform3d>(
          "Camera_1",
          new Transform3d(
            new Translation3d(Units.inchesToMeters(1.25), Units.inchesToMeters(7.125), Units.inchesToMeters(21)),
            new Rotation3d(0, 0, 0)
          )
        ),

        new Pair<String, Transform3d>(
          "Camera_2",
          new Transform3d(
            new Translation3d(Units.inchesToMeters(1.25), Units.inchesToMeters(-7.125), Units.inchesToMeters(21)),
            new Rotation3d(0, 0, 0)
          )
        )
      ));
    }
  }

  // TODO: check/tune vision weight
  // How much to trust vision measurements normally
  public static final Matrix<N3, N1> kBaseVisionPoseStdDevs = new MatBuilder<>(Nat.N3(), Nat.N1()).fill(
    0.9, // x in meters (default=0.9)
    0.9, // y in meters (default=0.9)
    0.9 // heading in radians (default=0.9)
  );

  public static final Matrix<N3, N1> kChargeStationVisionPoseStdDevs = new MatBuilder<>(Nat.N3(), Nat.N1()).fill(
    0.01, // x in meters
    0.01, // y in meters
    0.01 // heading in radians
  );

  // Increasing this makes pose estimation trust vision measurements less as distance from Apriltags increases
  // This is how much is added to std dev for vision when closest visible Apriltag is 1 meter away
   //TODO: how much? was 0.1 but richie had zero, WPI recommends not to include measurements >1 m, so maybe should be higher
  public static final double kVisionPoseStdDevFactor = 0.01;

  public static final double kFieldLength = Units.inchesToMeters(54*12 + 3.25);
  public static final double kFieldWidth = Units.inchesToMeters(26*12 + 3.5);

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
}