package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/**
 * Container class for general constants.
 */
public final class Constants {
  // RobotId key in rio preferences
  public static final String kRobotIdKey = "RobotId";

  public static final double kGravitationalAccel = 9.8;
  public static final double kMaxVoltage = 12.0;
  public static final double kLoopTime = 0.02;
  
  public static final double kCancoderResolution = 4096;

  // CAN bus names
  public static final String kCanivoreCAN = "CANivore";
  public static final String kRioCAN = "rio";

  public static final double kFieldLength = Units.inchesToMeters(54*12 + 3.25);
  public static final double kFieldWidth = Units.inchesToMeters(26*12 + 3.5);

  public static class Falcon {
    public static final double kResolution = 2048;
    public static final double kMaxRpm = 6380.0; // Rotations per minute
  }

  public static class Auto {
    public static final double kMaxAutoSpeed = 1.0; // m/s
    public static final double kMaxAutoAccel = 1.0; // m/s^2
  }

  public static class Vision {
    public static ArrayList<Pair<String, Transform3d>> kCameras = new ArrayList<Pair<String, Transform3d>>(List.of());

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

  public static class Drive {
    public static double kWheelRadius = Units.inchesToMeters(2);

    public static double kTrackWidth = Units.inchesToMeters(20.75);
    public static double kDriveGearRatio = 6.75;
    public static double kSteerGearRatio = 21.43;

    public static double kMaxSpeed = (Constants.Falcon.kMaxRpm / 60.0) * kDriveGearRatio * kWheelRadius * 2 * Math.PI;

    // Need to convert tangential velocity (the m/s of the edge of the robot) to angular velocity (the radians/s of the robot)
    // To do so, divide by the radius. The radius is the diagonal of the square chassis, diagonal = sqrt(2) * side_length.
    public static double kMaxAngularSpeed = kMaxSpeed / ((kTrackWidth/2) * Math.sqrt(2));

    // TODO: tune this better.
    public static double kMaxAngularAccel = 8 * 2 * Math.PI; // 8 rotations per second per second

    public static int kPigeon = 0;

    public static double kStartingHeadingDegrees = 0;

    // heading PID
    public static double kHeadingP= 4.6;
    public static double kHeadingI= 0;
    public static double kHeadingD= 0.4;

    // CAN
    public static String kDriveMotorCAN = Constants.kCanivoreCAN;
    public static String kSteerMotorCAN = Constants.kCanivoreCAN;
    public static String kSteerEncoderCAN = Constants.kCanivoreCAN;
    public static String kPigeonCAN = Constants.kCanivoreCAN;


    public static class Mod {
      public int kDrive, kSteer, kEncoder;
      public double kSteerOffset, kDriveKS, kDriveKV, kDriveP, kDriveI, kDriveD, kSteerKS, kSteerKV, kSteerP, kSteerI, kSteerD;
      public Mod(int drive, int steer, int encoder, double steerOffset, double driveKS, double driveKV, double driveP, double driveI,
                 double driveD, double steerKS, double steerKV, double steerP, double steerI, double steerD) {
        kDrive = drive;
        kSteer = steer;
        kEncoder = encoder;
        kSteerOffset = steerOffset;
        kDriveKS = driveKS;
        kDriveKV = driveKV;
        kDriveP = driveP;
        kDriveI = driveI;
        kDriveD = driveD;
        kSteerKS = steerKS;
        kSteerKV = steerKV;
        kSteerP = steerP;
        kSteerI = steerI;
        kSteerD = steerD;
      }
    }

    public static Mod FL = new Mod(20, 15, 40, 0.058291152119637, 0.5671488314798404, 4.546193650654074*2, 2*2, 0, 0, 0.6996691434004014, 0.3632874182531057, 14, 0, 0);
    public static Mod FR = new Mod(33, 30, 41, -2.994324445724487, 0.6880086692853459, 4.336917320413962*2, 2*2, 0, 0, 0.6429664500296258, 0.3645372486658138, 14, 0, 0);
    public static Mod BL = new Mod(16, 18, 42, -2.540267050266266, 0.4549680357672717, 4.936626078325219*2, 2*2, 0, 0, 0.6685253827543534, 0.3703123041002871, 14, 0, 0);
    public static Mod BR = new Mod(32, 35, 43, 2.626169800758362, 0.46914605136974696, 4.625248274107886*2, 2.2*2, 0, 0, 0.7151324607226578, 0.3731499810181726, 14, 0, 0);
  }

  public static void configureTestConstants() {
    Drive.kTrackWidth = Units.inchesToMeters(22.75);
    Drive.kMaxAngularSpeed = Drive.kMaxSpeed / ((Drive.kTrackWidth/2) * Math.sqrt(2));

    Drive.kPigeon = 13;

    Drive.kDriveMotorCAN = kRioCAN;

    Drive.FL = new Drive.Mod(1, 2, 3, 1.561, 0.61534, 4.45071*2, 2*2, 0, 0, 1, 0.5, 12, 0, 0);
    Drive.FR = new Drive.Mod(4,  5, 6,  -2.764+Math.PI, 0.61283, 4.45431*2, 2*2, 0, 0, 1, 0.5, 12, 0, 0);
    Drive.BL = new Drive.Mod(7, 8, 9,  0, 0.54150, 4.53909*2, 2*2, 0, 0, 1, 0.5, 12, 0, 0);
    Drive.BR = new Drive.Mod(10, 11, 12,  2.73, 0.49599, 4.75897*2, 2.2*2, 0, 0, 1, 0.5, 12, 0, 0);
      
    Vision.kCameras = new ArrayList<Pair<String, Transform3d>>(List.of(
      new Pair<String, Transform3d>(
        "Camera_2",
        new Transform3d(
          new Translation3d(-Units.inchesToMeters(4.75), Units.inchesToMeters(10.375), Units.inchesToMeters(10)),
          new Rotation3d(0, 0, 0)
        )
      )
    ));
  }
}
