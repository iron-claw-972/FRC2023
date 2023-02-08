package frc.robot.constants.swerve;

import edu.wpi.first.math.util.Units;
import frc.robot.constants.Constants;

public class TestDriveConstants extends DriveConstants {

    private static final double kWheelRadius = Units.inchesToMeters(2);

    private static final double kTrackWidth = Units.inchesToMeters(22.75);//22.75 swerve bot, 20.75 comp bot
    private static final double kDriveGearRatio = 6.75;
    private static final double kSteerGearRatio = 12.8;

    private static final double kMaxSpeed = Constants.falcon.kMaxRpm / 60.0 / kDriveGearRatio * kWheelRadius * 2 * Math.PI;

    private static final double kMaxAngularSpeed = Constants.falcon.kMaxRpm / 60.0 / kSteerGearRatio * 2 * Math.PI; // 8.3 rot/s
    private static final double kMaxAngularAccel = 8 * 2 * Math.PI; // 8 rotations per second per second

    private static final int kPigeon = 0;

    private static final double kStartingHeadingDegrees = 0;

    private static final int kDriveFrontLeft = 1;
    private static final int kSteerFrontLeft = 2;
    private static final int kEncoderFrontLeft = 3;
    private static final double kSteerOffsetFrontLeft = 1.561;
    private static final double kDriveKSFrontLeft = 0.61534;
    private static final double kDriveKVFrontLeft = 4.45071;
    private static final double kDrivePFrontLeft = 2;
    private static final double kDriveIFrontLeft = 0;
    private static final double kDriveDFrontLeft = 0;
    private static final double kSteerKSFrontLeft = 1;
    private static final double kSteerKVFrontLeft = 0.5;
    private static final double kSteerPFrontLeft = 2;
    private static final double kSteerIFrontLeft = 0;
    private static final double kSteerDFrontLeft = 0;

    private static final int kDriveFrontRight = 4; 
    private static final int kSteerFrontRight = 5;
    private static final int kEncoderFrontRight = 6; 
    private static final double kSteerOffsetFrontRight = -2.764+Math.PI;
    private static final double kDriveKSFrontRight = 0.61283;
    private static final double kDriveKVFrontRight = 4.45431;
    private static final double kDrivePFrontRight = 2;
    private static final double kDriveIFrontRight = 0;
    private static final double kDriveDFrontRight = 0;
    private static final double kSteerKSFrontRight = 1;
    private static final double kSteerKVFrontRight = 0.5;
    private static final double kSteerPFrontRight = 2;
    private static final double kSteerIFrontRight = 0;
    private static final double kSteerDFrontRight = 0;

    private static final int kDriveBackLeft = 7;
    private static final int kSteerBackLeft = 8;
    private static final int kEncoderBackLeft = 9; 
    private static final double kSteerOffsetBackLeft = 0;
    private static final double kDriveKSBackLeft = 0.54150;
    private static final double kDriveKVBackLeft = 4.53909;
    private static final double kDrivePBackLeft = 2;
    private static final double kDriveIBackLeft = 0;
    private static final double kDriveDBackLeft = 0;
    private static final double kSteerKSBackLeft = 1;
    private static final double kSteerKVBackLeft = 0.5;
    private static final double kSteerPBackLeft = 2;
    private static final double kSteerIBackLeft = 0;
    private static final double kSteerDBackLeft = 0;

    private static final int kDriveBackRight = 10;
    private static final int kSteerBackRight = 11;
    private static final int kEncoderBackRight = 12; 
    private static final double kSteerOffsetBackRight = 2.73;
    private static final double kDriveKSBackRight = 0.49599;
    private static final double kDriveKVBackRight = 4.75897;
    private static final double kDrivePBackRight = 2.2;
    private static final double kDriveIBackRight = 0;
    private static final double kDriveDBackRight = 0;
    private static final double kSteerKSBackRight = 1;
    private static final double kSteerKVBackRight = 0.5;
    private static final double kSteerPBackRight = 2;
    private static final double kSteerIBackRight = 0;
    private static final double kSteerDBackRight = 0;

    // PID
    // Drive
    private static final double kDrivePAll = 0;
    private static final double kDriveIAll = 0;
    private static final double kDriveDAll = 0;
    // Steer
    private static final double kSteerPAll = 12; //SDS: 0.2
    private static final double kSteerIAll = 0;
    private static final double kSteerDAll = 0; //SDS: 0.1

    private static final double kDriveKSAll = 0.63107;
    private static final double kDriveKVAll = 2.1592;

    // Steer
    private static final double kSteerKS = 1;
    private static final double kSteerKV = 0.5;
    // heading PID
    private static final double KheadingP= 4.6;
    private static final double KheadingI= 0;
    private static final double KheadingD= 0.4;

    // CAN
    private static final String kDriveMotorCAN = "rio";
    private static final String kSteerMotorCAN = "CANivore";

    public TestDriveConstants() {
        super(
            kWheelRadius,
            kTrackWidth,
            kDriveGearRatio,
            kSteerGearRatio,

            kMaxSpeed,
            kMaxAngularSpeed,
            kMaxAngularAccel,

            kPigeon,
            kStartingHeadingDegrees,

            kDriveFrontLeft,
            kSteerFrontLeft,
            kEncoderFrontLeft,
            kSteerOffsetFrontLeft,
            kDriveKSFrontLeft,
            kDriveKVFrontLeft,
            kDrivePFrontLeft,
            kDriveIFrontLeft,
            kDriveDFrontLeft,
            kSteerKSFrontLeft,
            kSteerKVFrontLeft,
            kSteerPFrontLeft,
            kSteerIFrontLeft,
            kSteerDFrontLeft,

            kDriveFrontRight,
            kSteerFrontRight,
            kEncoderFrontRight,
            kSteerOffsetFrontRight,
            kDriveKSFrontRight,
            kDriveKVFrontRight,
            kDrivePFrontRight,
            kDriveIFrontRight,
            kDriveDFrontRight,
            kSteerKSFrontRight,
            kSteerKVFrontRight,
            kSteerPFrontRight,
            kSteerIFrontRight,
            kSteerDFrontRight,
            
            kDriveBackLeft,
            kSteerBackLeft,
            kEncoderBackLeft,
            kSteerOffsetBackLeft,
            kDriveKSBackLeft,
            kDriveKVBackLeft,
            kDrivePBackLeft,
            kDriveIBackLeft,
            kDriveDBackLeft,
            kSteerKSBackLeft,
            kSteerKVBackLeft,
            kSteerPBackLeft,
            kSteerIBackLeft,
            kSteerDBackLeft,

            kDriveBackRight,
            kSteerBackRight,
            kEncoderBackRight,
            kSteerOffsetBackRight,
            kDriveKSBackRight,
            kDriveKVBackRight,
            kDrivePBackRight,
            kDriveIBackRight,
            kDriveDBackRight,
            kSteerKSBackRight,
            kSteerKVBackRight,
            kSteerPBackRight,
            kSteerIBackRight,
            kSteerDBackRight,

            kDrivePAll,
            kDriveIAll,
            kDriveDAll,
            kSteerPAll,
            kSteerIAll,
            kSteerDAll,
            kDriveKSAll,
            kDriveKVAll,
            kSteerKS,
            kSteerKV,
            KheadingP,
            KheadingI,
            KheadingD,

            kDriveMotorCAN,
            kSteerMotorCAN
        );
    }
    
}
