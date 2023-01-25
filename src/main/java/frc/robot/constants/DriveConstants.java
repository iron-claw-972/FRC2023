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

    public final int kDriveFrontLeft = 1;
    public final int kSteerFrontLeft = 2;
    public final int kEncoderFrontLeft = 3;
    public final double kSteerOffsetFrontLeft = 1.561;
    public final double kDriveKSFrontLeft = 0.61534;
    public final double kDriveKVFrontLeft = 4.45071;
    public final double kDrivePFrontLeft = 2;
    public final double kDriveIFrontLeft = 0;
    public final double kDriveDFrontLeft = 0;

    public final int kDriveFrontRight = 4; 
    public final int kSteerFrontRight = 5;
    public final int kEncoderFrontRight = 6; 
    public final double kSteerOffsetFrontRight = -2.764+Math.PI;
    public final double kDriveKSFrontRight = 0.61283;
    public final double kDriveKVFrontRight = 4.45431;
    public final double kDrivePFrontRight = 2;
    public final double kDriveIFrontRight = 0;
    public final double kDriveDFrontRight = 0;

    public final int kDriveBackLeft = 7;
    public final int kSteerBackLeft = 8;
    public final int kEncoderBackLeft = 9; 
    public final double kSteerOffsetBackLeft = 0;
    public final double kDriveKSBackLeft = 0.54150;
    public final double kDriveKVBackLeft = 4.53909;
    public final double kDrivePBackLeft = 2;
    public final double kDriveIBackLeft = 0;
    public final double kDriveDBackLeft = 0;

    public final int kDriveBackRight = 10;
    public final int kSteerBackRight = 11;
    public final int kEncoderBackRight = 12; 
    public final double kSteerOffsetBackRight = 2.73;
    public final double kDriveKSBackRight = 0.49599;
    public final double kDriveKVBackRight = 4.75897;
    public final double kDrivePBackRight = 2.2;
    public final double kDriveIBackRight = 0;
    public final double kDriveDBackRight = 0;

    // PID
    // Drive
    public final double kDriveP = 0;
    public final double kDriveI = 0;
    public final double kDriveD = 0;
    // Steer
    public final double kSteerP = 12; //SDS: 0.2
    public final double kSteerI = 0;
    public final double kSteerD = 0; //SDS: 0.1

    

    public final double kDriveKSAll = 0.63107;
    public final double kDriveKVAll = 2.1592;

    // Steer
    public final double kSteerKS = 1;
    public final double kSteerKV = 0.5;
}
