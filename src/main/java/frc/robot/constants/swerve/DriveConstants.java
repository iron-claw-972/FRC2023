package frc.robot.constants.swerve;

import edu.wpi.first.math.util.Units;
import frc.robot.Robot;
import frc.robot.Robot.RobotId;
import frc.robot.constants.Constants;
import frc.robot.constants.FalconConstants;
import frc.robot.util.RobotType;

public abstract class DriveConstants {

    public static double kWheelRadius = Units.inchesToMeters(2);

    public static double kTrackWidth = Units.inchesToMeters(22.75);//22.75 swerve bot, 20.75 comp bot
    public static double kDriveGearRatio = 6.75;
    public static double kSteerGearRatio = 12.8;

    public static double kMaxSpeed = FalconConstants.kMaxRpm / 60.0 / kDriveGearRatio * kWheelRadius * 2 * Math.PI;

    public static double kMaxAngularSpeed = FalconConstants.kMaxRpm / 60.0 / kSteerGearRatio * 2 * Math.PI; // 8.3 rot/s
    public static double kMaxAngularAccel = 8 * 2 * Math.PI; // 8 rotations per second per second

    public static int kPigeon = 0;

    public static double kStartingHeadingDegrees = 0;
    
    public static int kDriveFrontLeft = 20;
    public static int kSteerFrontLeft = 15;
    public static int kEncoderFrontLeft = 40;
    public static double kSteerOffsetFrontLeft = 0.058291152119637;//-3.060285486280918+Math.PI;
    public static double kDriveKSFrontLeft = 0.5671488314798404;
    public static double kDriveKVFrontLeft = 4.546193650654074;
    public static double kDrivePFrontLeft = 2;
    public static double kDriveIFrontLeft = 0;
    public static double kDriveDFrontLeft = 0;
    public static double kSteerKSFrontLeft = 0.6996691434004014;
    public static double kSteerKVFrontLeft = 0.3632874182531057;
    public static double kSteerPFrontLeft = 14;
    public static double kSteerIFrontLeft = 0;
    public static double kSteerDFrontLeft = 0;

    public static int kDriveFrontRight = 33; 
    public static int kSteerFrontRight = 30;
    public static int kEncoderFrontRight = 41; 
    public static double kSteerOffsetFrontRight = -2.994324445724487;//-3.001994334161282;
    public static double kDriveKSFrontRight = 0.6880086692853459;
    public static double kDriveKVFrontRight = 4.336917320413962;
    public static double kDrivePFrontRight = 2;
    public static double kDriveIFrontRight = 0;
    public static double kDriveDFrontRight = 0;
    public static double kSteerKSFrontRight = 0.6429664500296258;
    public static double kSteerKVFrontRight = 0.3645372486658138;
    public static double kSteerPFrontRight = 14;
    public static double kSteerIFrontRight = 0;
    public static double kSteerDFrontRight = 0;

    public static int kDriveBackLeft = 16;
    public static int kSteerBackLeft = 18;
    public static int kEncoderBackLeft = 42; 
    public static double kSteerOffsetBackLeft = -2.540267050266266;//0.650406539440155+Math.PI;
    public static double kDriveKSBackLeft = 0.4549680357672717;
    public static double kDriveKVBackLeft = 4.936626078325219;
    public static double kDrivePBackLeft = 2;
    public static double kDriveIBackLeft = 0;
    public static double kDriveDBackLeft = 0;
    public static double kSteerKSBackLeft = 0.6685253827543534;
    public static double kSteerKVBackLeft = 0.3703123041002871;
    public static double kSteerPBackLeft = 14;
    public static double kSteerIBackLeft = 0;
    public static double kSteerDBackLeft = 0;

    public static int kDriveBackRight = 32;
    public static int kSteerBackRight = 35;
    public static int kEncoderBackRight = 43; 
    public static double kSteerOffsetBackRight = 2.626169800758362;//2.771897681057453;
    public static double kDriveKSBackRight = 0.46914605136974696;
    public static double kDriveKVBackRight = 4.625248274107886;
    public static double kDrivePBackRight = 2.2;
    public static double kDriveIBackRight = 0;
    public static double kDriveDBackRight = 0;
    public static double kSteerKSBackRight = 0.7151324607226578;
    public static double kSteerKVBackRight = 0.3731499810181726;
    public static double kSteerPBackRight = 14;
    public static double kSteerIBackRight = 0;
    public static double kSteerDBackRight = 0;
    
    // Drive
    public static double kDrivePAll = 0;
    public static double kDriveIAll = 0;
    public static double kDriveDAll = 0;
    // Steer
    public static double kSteerPAll = 12; //SDS: 0.2
    public static double kSteerIAll = 0;
    public static double kSteerDAll = 0; //SDS: 0.1

    public static double kDriveKSAll = 0.63107;
    public static double kDriveKVAll = 2.1592;

    // Steer
    public static double kSteerKS = 1;
    public static double kSteerKV = 0.5;
    // heading PID
    public static double KheadingP= 4.6;
    public static double KheadingI= 0;
    public static double KheadingD= 0.4;

    // CAN
    public static String kDriveMotorCAN = "CANivore";
    public static String kSteerMotorCAN = "CANivore";
    public static String kPigeonCAN = "CANivore";
    public static String kEncoderCAN = "CANivore";

    static {
        if (Robot.kRobotId == RobotId.SwerveTest) {
            kWheelRadius = Units.inchesToMeters(2);

            kTrackWidth = Units.inchesToMeters(22.75);//22.75 swerve bot, 20.75 comp bot

            kPigeon = 13;

            kDriveFrontLeft = 1;
            kSteerFrontLeft = 2;
            kEncoderFrontLeft = 3;
            kSteerOffsetFrontLeft = 1.561;
            kDriveKSFrontLeft = 0.61534;
            kDriveKVFrontLeft = 4.45071;
            kDrivePFrontLeft = 2;
            kDriveIFrontLeft = 0;
            kDriveDFrontLeft = 0;
            kSteerKSFrontLeft = 1;
            kSteerKVFrontLeft = 0.5;
            kSteerPFrontLeft = 12;
            kSteerIFrontLeft = 0;
            kSteerDFrontLeft = 0;

            kDriveFrontRight = 4; 
            kSteerFrontRight = 5;
            kEncoderFrontRight = 6; 
            kSteerOffsetFrontRight = -2.764+Math.PI;
            kDriveKSFrontRight = 0.61283;
            kDriveKVFrontRight = 4.45431;
            kDrivePFrontRight = 2;
            kDriveIFrontRight = 0;
            kDriveDFrontRight = 0;
            kSteerKSFrontRight = 1;
            kSteerKVFrontRight = 0.5;
            kSteerPFrontRight = 12;
            kSteerIFrontRight = 0;
            kSteerDFrontRight = 0;

            kDriveBackLeft = 7;
            kSteerBackLeft = 8;
            kEncoderBackLeft = 9; 
            kSteerOffsetBackLeft = 0;
            kDriveKSBackLeft = 0.54150;
            kDriveKVBackLeft = 4.53909;
            kDrivePBackLeft = 2;
            kDriveIBackLeft = 0;
            kDriveDBackLeft = 0;
            kSteerKSBackLeft = 1;
            kSteerKVBackLeft = 0.5;
            kSteerPBackLeft = 12;
            kSteerIBackLeft = 0;
            kSteerDBackLeft = 0;

            kDriveBackRight = 10;
            kSteerBackRight = 11;
            kEncoderBackRight = 12; 
            kSteerOffsetBackRight = 2.73;
            kDriveKSBackRight = 0.49599;
            kDriveKVBackRight = 4.75897;
            kDrivePBackRight = 2.2;
            kDriveIBackRight = 0;
            kDriveDBackRight = 0;
            kSteerKSBackRight = 1;
            kSteerKVBackRight = 0.5;
            kSteerPBackRight = 12;
            kSteerIBackRight = 0;
            kSteerDBackRight = 0;

            // Drive
            kDrivePAll = 0;
            kDriveIAll = 0;
            kDriveDAll = 0;
            // Steer
            kSteerPAll = 12; //SDS: 0.2
            kSteerIAll = 0;
            kSteerDAll = 0; //SDS: 0.1

            kDriveKSAll = 0.63107;
            kDriveKVAll = 2.1592;

            // Steer
            kSteerKS = 1;
            kSteerKV = 0.5;
            // heading PID
            KheadingP= 4.6;
            KheadingI= 0;
            KheadingD= 0.4;

            // CAN
            kDriveMotorCAN = "rio";
        }
    }

}
