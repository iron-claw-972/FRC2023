package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public class DriveConstants {

    public final double kWheelRadius = Units.inchesToMeters(2);

    public final double kTrackWidth = Units.inchesToMeters(22.75);
    public final double kDriveGearRatio = 6.75;
    public final double kSteerGearRatio = 12.8;

    public final double kMaxSpeed = Constants.kFalconMaxRPM / 60.0 / kDriveGearRatio * kWheelRadius * 2 * Math.PI;

    public final double kMaxAngularSpeed = Constants.kFalconMaxRPM / 60.0 / kSteerGearRatio * 2 * Math.PI; // 8.3 rot/s
    public final double kMaxAngularAccel = 8 * 2 * Math.PI; // 8 rotations per second per second

    public final int kPigeon = 13;

    public final double kStartingHeadingDegrees = 0;

    public final int kDriveFrontLeft = 1;
    public final int kSteerFrontLeft = 2;
    public final int kEncoderFrontLeft = 3;
    public final double kSteerOffsetFrontLeft = 1.561;

    public final int kDriveFrontRight = 4; 
    public final int kSteerFrontRight = 5;
    public final int kEncoderFrontRight = 6; 
    public final double kSteerOffsetFrontRight = -1.179;

    public final int kDriveBackLeft = 7; 
    public final int kSteerBackLeft = 8;
    public final int kEncoderBackLeft = 9; 
    public final double kSteerOffsetBackLeft = 0;

    public final int kDriveBackRight = 10;
    public final int kSteerBackRight = 11;
    public final int kEncoderBackRight = 12; 
    public final double kSteerOffsetBackRight = 2.73; 

    // PID
    // Drive
    public final double kDriveP = 0;
    public final double kDriveI = 0;
    public final double kDriveD = 0;
    // Steer
    public final double kSteerP = 12; //SDS: 0.2
    public final double kSteerI = 0;
    public final double kSteerD = 0; //SDS: 0.1

    // FEEDFORWARD
    // Drive
    public final double kDriveKS = 0.2;
    public final double kDriveKV = 2.7;
    // Steer
    public final double kSteerKS = 1;
    public final double kSteerKV = 0.5;
}
