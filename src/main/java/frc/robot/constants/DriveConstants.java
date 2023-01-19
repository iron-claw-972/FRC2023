package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public class DriveConstants {

    public final double kWheelRadius = Units.inchesToMeters(2);

    public final double kTrackWidth = Units.inchesToMeters(22.75);
    public final double kDriveGearRatio = 6.75;
    public final double kSteerGearRatio = 12.8;

    public final double kMaxSpeed = Constants.falcon.kMaxRpm / 60.0 / kDriveGearRatio * kWheelRadius * 2 * Math.PI;

    public final double kMaxAngularSpeed = Constants.falcon.kMaxRpm / 60.0 / kSteerGearRatio * 2 * Math.PI; // 8.3 rot/s
    public final double kMaxAngularAccel = 8 * 2 * Math.PI; // 8 rotations per second per second

    public final int kPigeon = 13;

    public final double kStartingHeadingDegrees = 0;

    public final int kDriveFrontLeft = 20;
    public final int kSteerFrontLeft = 31;
    public final int kEncoderFrontLeft = 3;
    public final double kSteerOffsetFrontLeft = 1.561;
    public final double kDriveKSFrontLeft = 0.63107;
    public final double kDriveKVFrontLeft = 2.1592;

    public final int kDriveFrontRight = 30; 
    public final int kSteerFrontRight = 33;
    public final int kEncoderFrontRight = 6; 
    public final double kSteerOffsetFrontRight = -2.764+Math.PI;
    public final double kDriveKSFrontRight = 0.63107;
    public final double kDriveKVFrontRight = 2.1592;

    public final int kDriveBackLeft = 16; 
    public final int kSteerBackLeft = 8;
    public final int kEncoderBackLeft = 9; 
    public final double kSteerOffsetBackLeft = 0;
    public final double kDriveKSBackLeft = 0.63107;
    public final double kDriveKVBackLeft = 2.1592;

    public final int kDriveBackRight = 32;
    public final int kSteerBackRight = 35;
    public final int kEncoderBackRight = 12; 
    public final double kSteerOffsetBackRight = 2.73;
    public final double kDriveKSBackRight = 0.63107;
    public final double kDriveKVBackRight = 2.1592;

    // PID
    // Drive
    public final double kDriveP = 0;
    public final double kDriveI = 0;
    public final double kDriveD = 0;
    // Steer
    public final double kSteerP = 12; //SDS: 0.2
    public final double kSteerI = 0;
    public final double kSteerD = 0; //SDS: 0.1

    

    public final double kDriveAllKS = 0.63107;
    public final double kDriveAllKV = 2.1592;

    // Steer
    public final double kSteerKS = 1;
    public final double kSteerKV = 0.5;
}
