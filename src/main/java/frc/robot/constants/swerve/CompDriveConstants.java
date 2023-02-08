package frc.robot.constants.swerve;

import edu.wpi.first.math.util.Units;
import frc.robot.constants.Constants;

public class CompDriveConstants extends DriveConstants {
    private static final double kWheelRadius = Units.inchesToMeters(2);

    private static final double kTrackWidth = Units.inchesToMeters(20.75);//22.75 swerve bot, 20.75 comp bot
    private static final double kDriveGearRatio = 6.75;
    private static final double kSteerGearRatio = 12.8;

    private static final double kMaxSpeed = Constants.falcon.kMaxRpm / 60.0 / kDriveGearRatio * kWheelRadius * 2 * Math.PI;

    private static final double kMaxAngularSpeed = Constants.falcon.kMaxRpm / 60.0 / kSteerGearRatio * 2 * Math.PI; // 8.3 rot/s
    private static final double kMaxAngularAccel = 8 * 2 * Math.PI; // 8 rotations per second per second

    private static final int kPigeon = 0;

    private static final double kStartingHeadingDegrees = 0;

    
    private static final int kDriveFrontLeft = 20;
    private static final int kSteerFrontLeft = 15;
    private static final int kEncoderFrontLeft = 40;
    private static final double kSteerOffsetFrontLeft = -3.060285486280918+Math.PI;
    private static final double kDriveKSFrontLeft = 0.5671488314798404;
    private static final double kDriveKVFrontLeft = 4.546193650654074;
    private static final double kDrivePFrontLeft = 2;
    private static final double kDriveIFrontLeft = 0;
    private static final double kDriveDFrontLeft = 0;
    private static final double kSteerKSFrontLeft = 0.6125097737272045;
    private static final double kSteerKVFrontLeft = 0.36443278707381527;
    private static final double kSteerPFrontLeft = 2;
    private static final double kSteerIFrontLeft = 0;
    private static final double kSteerDFrontLeft = 0;

    private static final int kDriveFrontRight = 33; 
    private static final int kSteerFrontRight = 30;
    private static final int kEncoderFrontRight = 41; 
    private static final double kSteerOffsetFrontRight = -3.001994334161282;
    private static final double kDriveKSFrontRight = 0.6003299362722261;
    private static final double kDriveKVFrontRight = 4.336917320413962;
    private static final double kDrivePFrontRight = 2;
    private static final double kDriveIFrontRight = 0;
    private static final double kDriveDFrontRight = 0;
    private static final double kSteerKSFrontRight = 0.6791531799792174;
    private static final double kSteerKVFrontRight = 0.3656397076138848;
    private static final double kSteerPFrontRight = 2;
    private static final double kSteerIFrontRight = 0;
    private static final double kSteerDFrontRight = 0;

    private static final int kDriveBackLeft = 16;
    private static final int kSteerBackLeft = 18;
    private static final int kEncoderBackLeft = 42; 
    private static final double kSteerOffsetBackLeft = 0.650406539440155+Math.PI;
    private static final double kDriveKSBackLeft = 0.4549680357672717;
    private static final double kDriveKVBackLeft = 4.936626078325219;
    private static final double kDrivePBackLeft = 2;
    private static final double kDriveIBackLeft = 0;
    private static final double kDriveDBackLeft = 0;

    private static final int kDriveBackRight = 32;
    private static final int kSteerBackRight = 35;
    private static final int kEncoderBackRight = 43; 
    private static final double kSteerOffsetBackRight = 2.771897681057453;
    private static final double kDriveKSBackRight = 0.46914605136974696;
    private static final double kDriveKVBackRight = 4.625248274107886;
    private static final double kDrivePBackRight = 2.2;
    private static final double kDriveIBackRight = 0;
    private static final double kDriveDBackRight = 0;
    private static final double kSteerKSBackLeft = 0.6655795947354465;
    private static final double kSteerKVBackLeft = 0.37272327600410865;
    private static final double kSteerPBackLeft = 2;
    private static final double kSteerIBackLeft = 0;
    private static final double kSteerDBackLeft = 0;
    private static final double kSteerKSBackRight = 0.6212531484419491;
    private static final double kSteerKVBackRight = 0.3742469253515487;
    private static final double kSteerPBackRight = 2;
    private static final double kSteerIBackRight = 0;
    private static final double kSteerDBackRight = 0;
    

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
    private static final String kDriveMotorCAN = "CANivore";
    private static final String kSteerMotorCAN = "CANivore";
    
    public CompDriveConstants() {
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
