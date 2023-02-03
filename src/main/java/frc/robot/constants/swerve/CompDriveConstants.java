package frc.robot.constants.swerve;

import edu.wpi.first.math.util.Units;
import frc.robot.constants.Constants;

public class CompDriveConstants {
    public static final double kWheelRadius = Units.inchesToMeters(2);

    public static final double kTrackWidth = Units.inchesToMeters(20.75);//22.75 swerve bot, 20.75 comp bot
    public static final double kDriveGearRatio = 6.75;
    public static final double kSteerGearRatio = 12.8;

    public static final double kMaxSpeed = Constants.falcon.kMaxRpm / 60.0 / kDriveGearRatio * kWheelRadius * 2 * Math.PI;

    public static final double kMaxAngularSpeed = Constants.falcon.kMaxRpm / 60.0 / kSteerGearRatio * 2 * Math.PI; // 8.3 rot/s
    public static final double kMaxAngularAccel = 8 * 2 * Math.PI; // 8 rotations per second per second

    public static final int kPigeon = 0;

    public static final double kStartingHeadingDegrees = 0;

    
    public static final int kDriveFrontLeft = 20;
    public static final int kSteerFrontLeft = 15;
    public static final int kEncoderFrontLeft = 40;
    public static final double kSteerOffsetFrontLeft = -3.060285486280918+Math.PI;
    public static final double kDriveKSFrontLeft = 0.5671488314798404;
    public static final double kDriveKVFrontLeft = 4.546193650654074;
    public static final double kDrivePFrontLeft = 2;
    public static final double kDriveIFrontLeft = 0;
    public static final double kDriveDFrontLeft = 0;
    public static final double kSteerKSFrontLeft = 0.6125097737272045;
    public static final double kSteerKVFrontLeft = 0.36443278707381527;
    public static final double kSteerPFrontLeft = 2;
    public static final double kSteerIFrontLeft = 0;
    public static final double kSteerDFrontLeft = 0;

    public static final int kDriveFrontRight = 33; 
    public static final int kSteerFrontRight = 30;
    public static final int kEncoderFrontRight = 41; 
    public static final double kSteerOffsetFrontRight = -3.001994334161282;
    public static final double kDriveKSFrontRight = 0.6003299362722261;
    public static final double kDriveKVFrontRight = 4.336917320413962;
    public static final double kDrivePFrontRight = 2;
    public static final double kDriveIFrontRight = 0;
    public static final double kDriveDFrontRight = 0;
    public static final double kSteerKSFrontRight = 0.6791531799792174;
    public static final double kSteerKVFrontRight = 0.3656397076138848;
    public static final double kSteerPFrontRight = 2;
    public static final double kSteerIFrontRight = 0;
    public static final double kSteerDFrontRight = 0;

    public static final int kDriveBackLeft = 16;
    public static final int kSteerBackLeft = 18;
    public static final int kEncoderBackLeft = 42; 
    public static final double kSteerOffsetBackLeft = 0.650406539440155+Math.PI;
    public static final double kDriveKSBackLeft = 0.4549680357672717;
    public static final double kDriveKVBackLeft = 4.936626078325219;
    public static final double kDrivePBackLeft = 2;
    public static final double kDriveIBackLeft = 0;
    public static final double kDriveDBackLeft = 0;

    public static final int kDriveBackRight = 32;
    public static final int kSteerBackRight = 35;
    public static final int kEncoderBackRight = 43; 
    public static final double kSteerOffsetBackRight = 2.771897681057453;
    public static final double kDriveKSBackRight = 0.46914605136974696;
    public static final double kDriveKVBackRight = 4.625248274107886;
    public static final double kDrivePBackRight = 2.2;
    public static final double kDriveIBackRight = 0;
    public static final double kDriveDBackRight = 0;
    public static final double kSteerKSBackLeft = 0.6655795947354465;
    public static final double kSteerKVBackLeft = 0.37272327600410865;
    public static final double kSteerPBackLeft = 2;
    public static final double kSteerIBackLeft = 0;
    public static final double kSteerDBackLeft = 0;
    public static final double kSteerKSBackRight = 0.6212531484419491;
    public static final double kSteerKVBackRight = 0.3742469253515487;
    public static final double kSteerPBackRight = 2;
    public static final double kSteerIBackRight = 0;
    public static final double kSteerDBackRight = 0;
    

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