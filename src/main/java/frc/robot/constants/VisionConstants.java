package frc.robot.constants;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.Robot.RobotId;
import frc.robot.constants.swerve.DriveConstants;

/**
 * Container class for vision constants.
 */
public class VisionConstants {
  // If vision is enabled
  public static final boolean kEnabled = true;

  //TODO: Change these to whatever the actual distances are
  // How far from the grid the robot should be to score for alignment in meters
  public static double kGridDistanceAlignment = 0 + DriveConstants.kRobotWidthWithBumpers / 2; // meters
  // How far from the shelf the robot should be to intake for alignment in meters
  public static double kShelfDistanceAlignment = 0.5 + DriveConstants.kRobotWidthWithBumpers / 2;

  public static Pose2d kBlueShelfAlignPose = new Pose2d(FieldConstants.kBlueShelfX - kShelfDistanceAlignment, FieldConstants.kShelfY, new Rotation2d(0));
  public static Pose2d kRedShelfAlignPose = new Pose2d(FieldConstants.kRedShelfX + kShelfDistanceAlignment, FieldConstants.kShelfY, new Rotation2d(Math.PI));

  public static double kStdDevCommandEndTime = 5;

  public static ArrayList<Pair<String, Transform3d>> kCameras = new ArrayList<>();

  /**
   * Update the camera list based on the robot id detected from rio preferences.
   */
  public static void update(RobotId robotID) {
    if (robotID == RobotId.SwerveTest) {
      kCameras = new ArrayList<Pair<String, Transform3d>>(List.of(
        new Pair<String, Transform3d>(
          "Camera_2",
          new Transform3d(
            new Translation3d(Units.inchesToMeters(15), Units.inchesToMeters(-3.3125), Units.inchesToMeters(10)),
            new Rotation3d(0, 0, Math.PI)
          )
        )
      ));
    } else {
      // DO NOT COMMENT THIS OUT
      kCameras = new ArrayList<Pair<String, Transform3d>>(List.of(
        new Pair<String, Transform3d>(
          "Left_Camera",
          new Transform3d(
            new Translation3d(Units.inchesToMeters(1.25 + 8.25), Units.inchesToMeters(7.125 - 0.25), Units.inchesToMeters(21 + 16)),
            new Rotation3d(Math.PI, Units.degreesToRadians(-18), 0)
          ))
        ,
        new Pair<String, Transform3d>(
          "Right_Camera",
          new Transform3d(
            new Translation3d(Units.inchesToMeters(-0.75), Units.inchesToMeters(-7.125), Units.inchesToMeters(21)),
            new Rotation3d(0, 0, Math.PI)
          )
        )
      ));
    }
  }

  /** Poses that use targets with an ambiguity above this amount will be ignored */
  public static final double highestAmbiguity = 0.04;

  // TODO: check/tune vision weight
  // How much to trust vision measurements normally
  public static final Matrix<N3, N1> kBaseVisionPoseStdDevs = new MatBuilder<>(Nat.N3(), Nat.N1()).fill(
    0.5, // x in meters (default=0.9)
    0.5, // y in meters (default=0.9)
    10000 // heading in radians (default=0.9)
  );

  public static final Matrix<N3, N1> kChargeStationVisionPoseStdDevs = new MatBuilder<>(Nat.N3(), Nat.N1()).fill(
    0.5, // x in meters
    0.5, // y in meters
    10000 // heading in radians
  );

  // Increasing this makes pose estimation trust vision measurements less as distance from Apriltags increases
  // This is how much is added to std dev for vision when closest visible Apriltag is 1 meter away
  public static final double kVisionPoseStdDevFactor = 0.5;
}