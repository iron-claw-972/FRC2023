package frc.robot.constants.swerve;

import edu.wpi.first.math.util.Units;
import frc.robot.constants.Constants;

public class TestDriveConstants {

    public static final double kWheelRadius = Units.inchesToMeters(2);

    public static final double kTrackWidth = Units.inchesToMeters(22.75);//22.75 swerve bot, 20.75 comp bot
    public static final double kDriveGearRatio = 6.75;
    public static final double kSteerGearRatio = 12.8;

    public static final double kMaxSpeed = Constants.falcon.kMaxRpm / 60.0 / kDriveGearRatio * kWheelRadius * 2 * Math.PI;

    public static final double kMaxAngularSpeed = Constants.falcon.kMaxRpm / 60.0 / kSteerGearRatio * 2 * Math.PI; // 8.3 rot/s
    public static final double kMaxAngularAccel = 8 * 2 * Math.PI; // 8 rotations per second per second

    public static final int kPigeon = 0;

    public static final double kStartingHeadingDegrees = 0;

    public static final int kDriveFrontLeft = 1;
    public static final int kSteerFrontLeft = 2;
    public static final int kEncoderFrontLeft = 3;
    public static final double kSteerOffsetFrontLeft = 1.561;
    public static final double kDriveKSFrontLeft = 0.61534;
    public static final double kDriveKVFrontLeft = 4.45071;
    public static final double kDrivePFrontLeft = 2;
    public static final double kDriveIFrontLeft = 0;
    public static final double kDriveDFrontLeft = 0;
    public static final double kSteerKSFrontLeft = 1;
    public static final double kSteerKVFrontLeft = 0.5;
    public static final double kSteerPFrontLeft = 2;
    public static final double kSteerIFrontLeft = 0;
    public static final double kSteerDFrontLeft = 0;

    public static final int kDriveFrontRight = 4; 
    public static final int kSteerFrontRight = 5;
    public static final int kEncoderFrontRight = 6; 
    public static final double kSteerOffsetFrontRight = -2.764+Math.PI;
    public static final double kDriveKSFrontRight = 0.61283;
    public static final double kDriveKVFrontRight = 4.45431;
    public static final double kDrivePFrontRight = 2;
    public static final double kDriveIFrontRight = 0;
    public static final double kDriveDFrontRight = 0;
    public static final double kSteerKSFrontRight = 1;
    public static final double kSteerKVFrontRight = 0.5;
    public static final double kSteerPFrontRight = 2;
    public static final double kSteerIFrontRight = 0;
    public static final double kSteerDFrontRight = 0;

    public static final int kDriveBackLeft = 7;
    public static final int kSteerBackLeft = 8;
    public static final int kEncoderBackLeft = 9; 
    public static final double kSteerOffsetBackLeft = 0;
    public static final double kDriveKSBackLeft = 0.54150;
    public static final double kDriveKVBackLeft = 4.53909;
    public static final double kDrivePBackLeft = 2;
    public static final double kDriveIBackLeft = 0;
    public static final double kDriveDBackLeft = 0;
    public static final double kSteerKSBackLeft = 1;
    public static final double kSteerKVBackLeft = 0.5;
    public static final double kSteerPBackLeft = 2;
    public static final double kSteerIBackLeft = 0;
    public static final double kSteerDBackLeft = 0;

    public static final int kDriveBackRight = 10;
    public static final int kSteerBackRight = 11;
    public static final int kEncoderBackRight = 12; 
    public static final double kSteerOffsetBackRight = 2.73;
    public static final double kDriveKSBackRight = 0.49599;
    public static final double kDriveKVBackRight = 4.75897;
    public static final double kDrivePBackRight = 2.2;
    public static final double kDriveIBackRight = 0;
    public static final double kDriveDBackRight = 0;
    public static final double kSteerKSBackRight = 1;
    public static final double kSteerKVBackRight = 0.5;
    public static final double kSteerPBackRight = 2;
    public static final double kSteerIBackRight = 0;
    public static final double kSteerDBackRight = 0;

    // PID
    // Drive
    public static final double kDrivePAll = 0;
    public static final double kDriveIAll = 0;
    public static final double kDriveDAll = 0;
    // Steer
    public static final double kSteerPAll = 12; //SDS: 0.2
    public static final double kSteerIAll = 0;
    public static final double kSteerDAll = 0; //SDS: 0.1

    public static final double kDriveKSAll = 0.63107;
    public static final double kDriveKVAll = 2.1592;

    // Steer
    public static final double kSteerKS = 1;
    public static final double kSteerKV = 0.5;
    // heading PID
    public static final double KheadingP= 4.6;
    public static final double KheadingI= 0;
    public static final double KheadingD= 0.4;
    
}
