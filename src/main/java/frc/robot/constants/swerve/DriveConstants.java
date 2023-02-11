package frc.robot.constants.swerve;

public abstract class DriveConstants {

    public static final DriveConstants kConstants = new CompDriveConstants();
    
    public double kWheelRadius;

    public double kTrackWidth;
    public double kDriveGearRatio;
    public double kSteerGearRatio;

    public double kMaxSpeed;

    public double kMaxAngularSpeed;
    public double kMaxAngularAccel;

    public int kPigeon;

    public double kStartingHeadingDegrees;

    public int kDriveFrontLeft;
    public int kSteerFrontLeft;
    public int kEncoderFrontLeft;
    public double kSteerOffsetFrontLeft;
    public double kDriveKSFrontLeft;
    public double kDriveKVFrontLeft;
    public double kDrivePFrontLeft;
    public double kDriveIFrontLeft;
    public double kDriveDFrontLeft;
    public double kSteerKSFrontLeft;
    public double kSteerKVFrontLeft;
    public double kSteerPFrontLeft;
    public double kSteerIFrontLeft;
    public double kSteerDFrontLeft;

    public int kDriveFrontRight; 
    public int kSteerFrontRight;
    public int kEncoderFrontRight; 
    public double kSteerOffsetFrontRight;
    public double kDriveKSFrontRight;
    public double kDriveKVFrontRight;
    public double kDrivePFrontRight;
    public double kDriveIFrontRight;
    public double kDriveDFrontRight;
    public double kSteerKSFrontRight;
    public double kSteerKVFrontRight;
    public double kSteerPFrontRight;
    public double kSteerIFrontRight;
    public double kSteerDFrontRight;

    public int kDriveBackLeft;
    public int kSteerBackLeft;
    public int kEncoderBackLeft; 
    public double kSteerOffsetBackLeft;
    public double kDriveKSBackLeft;
    public double kDriveKVBackLeft;
    public double kDrivePBackLeft;
    public double kDriveIBackLeft;
    public double kDriveDBackLeft;
    public double kSteerKSBackLeft;
    public double kSteerKVBackLeft;
    public double kSteerPBackLeft;
    public double kSteerIBackLeft;
    public double kSteerDBackLeft;

    public int kDriveBackRight;
    public int kSteerBackRight;
    public int kEncoderBackRight; 
    public double kSteerOffsetBackRight;
    public double kDriveKSBackRight;
    public double kDriveKVBackRight;
    public double kDrivePBackRight;
    public double kDriveIBackRight;
    public double kDriveDBackRight;
    public double kSteerKSBackRight;
    public double kSteerKVBackRight;
    public double kSteerPBackRight;
    public double kSteerIBackRight;
    public double kSteerDBackRight;

    // Drive
    public double kDrivePAll;
    public double kDriveIAll;
    public double kDriveDAll;
    
    // Steer
    public double kSteerPAll; //SDS: 0.2 - maunfacturer recomended values
    public double kSteerIAll;
    public double kSteerDAll; //SDS: 0.1 - maunfacturer recomended values

    public double kDriveKSAll;
    public double kDriveKVAll;

    // Steer
    public double kSteerKS;
    public double kSteerKV;

    // heading PID
    public double KheadingP;
    public double KheadingI;
    public double KheadingD;

    // CAN
    public String kDriveMotorCAN;
    public String kSteerMotorCAN;
    public String kPigeonCAN;
    public String kEncoderCAN;

    public DriveConstants(
        double kWheelRadius,

        double kTrackWidth,
        double kDriveGearRatio,
        double kSteerGearRatio,

        double kMaxSpeed,

        double kMaxAngularSpeed,
        double kMaxAngularAccel,

        int kPigeon,

        double kStartingHeadingDegrees,

        int kDriveFrontLeft,
        int kSteerFrontLeft,
        int kEncoderFrontLeft,
        double kSteerOffsetFrontLeft,
        double kDriveKSFrontLeft,
        double kDriveKVFrontLeft,
        double kDrivePFrontLeft,
        double kDriveIFrontLeft,
        double kDriveDFrontLeft,
        double kSteerKSFrontLeft,
        double kSteerKVFrontLeft,
        double kSteerPFrontLeft,
        double kSteerIFrontLeft,
        double kSteerDFrontLeft,

        int kDriveFrontRight, 
        int kSteerFrontRight,
        int kEncoderFrontRight, 
        double kSteerOffsetFrontRight,
        double kDriveKSFrontRight,
        double kDriveKVFrontRight,
        double kDrivePFrontRight,
        double kDriveIFrontRight,
        double kDriveDFrontRight,
        double kSteerKSFrontRight,
        double kSteerKVFrontRight,
        double kSteerPFrontRight,
        double kSteerIFrontRight,
        double kSteerDFrontRight,

        int kDriveBackLeft,
        int kSteerBackLeft,
        int kEncoderBackLeft, 
        double kSteerOffsetBackLeft,
        double kDriveKSBackLeft,
        double kDriveKVBackLeft,
        double kDrivePBackLeft,
        double kDriveIBackLeft,
        double kDriveDBackLeft,
        double kSteerKSBackLeft,
        double kSteerKVBackLeft,
        double kSteerPBackLeft,
        double kSteerIBackLeft,
        double kSteerDBackLeft,

        int kDriveBackRight,
        int kSteerBackRight,
        int kEncoderBackRight, 
        double kSteerOffsetBackRight,
        double kDriveKSBackRight,
        double kDriveKVBackRight,
        double kDrivePBackRight,
        double kDriveIBackRight,
        double kDriveDBackRight,
        double kSteerKSBackRight,
        double kSteerKVBackRight,
        double kSteerPBackRight,
        double kSteerIBackRight,
        double kSteerDBackRight,

        // Drive
        double kDrivePAll,
        double kDriveIAll,
        double kDriveDAll,
        
        // Steer
        double kSteerPAll, //SDS: 0.2 - maunfacturer recomended values
        double kSteerIAll,
        double kSteerDAll, //SDS: 0.1 - maunfacturer recomended values

        double kDriveKSAll,
        double kDriveKVAll,

        // Steer
        double kSteerKS,
        double kSteerKV,

        // heading PID
        double KheadingP,
        double KheadingI,
        double KheadingD,

        // CAN
        String kDriveMotorCAN,
        String kSteerMotorCAN,
        String kPigeonCan,
        String kEncoderCAN
    ) {
        this.kWheelRadius = kWheelRadius;

        this.kTrackWidth = kTrackWidth;
        this.kDriveGearRatio = kDriveGearRatio;
        this.kSteerGearRatio = kSteerGearRatio;

        this.kMaxSpeed = kMaxSpeed;

        this.kMaxAngularSpeed = kMaxAngularSpeed;
        this.kMaxAngularAccel = kMaxAngularAccel;

        this.kPigeon = kPigeon;

        this.kStartingHeadingDegrees = kStartingHeadingDegrees;

        this.kDriveFrontLeft = kDriveFrontLeft;
        this.kSteerFrontLeft = kSteerFrontLeft;
        this.kEncoderFrontLeft = kEncoderFrontLeft;
        this.kSteerOffsetFrontLeft = kSteerOffsetFrontLeft;
        this.kDriveKSFrontLeft = kDriveKSFrontLeft;
        this.kDriveKVFrontLeft = kDriveKVFrontLeft;
        this.kDrivePFrontLeft = kDrivePFrontLeft;
        this.kDriveIFrontLeft = kDriveIFrontLeft;
        this.kDriveDFrontLeft = kDriveDFrontLeft;
        this.kSteerKSFrontLeft = kSteerKSFrontLeft;
        this.kSteerKVFrontLeft = kSteerKVFrontLeft;
        this.kSteerPFrontLeft = kSteerPFrontLeft;
        this.kSteerIFrontLeft = kSteerIFrontLeft;
        this.kSteerDFrontLeft = kSteerDFrontLeft;

        this.kDriveFrontRight = kDriveFrontRight;
        this.kSteerFrontRight = kSteerFrontRight;
        this.kEncoderFrontRight = kEncoderFrontRight;
        this.kSteerOffsetFrontRight = kSteerOffsetFrontRight;
        this.kDriveKSFrontRight = kDriveKSFrontRight;
        this.kDriveKVFrontRight = kDriveKVFrontRight;
        this.kDrivePFrontRight = kDrivePFrontRight;
        this.kDriveIFrontRight = kDriveIFrontRight;
        this.kDriveDFrontRight = kDriveDFrontRight;
        this.kSteerKSFrontRight = kSteerKSFrontRight;
        this.kSteerKVFrontRight = kSteerKVFrontRight;
        this.kSteerPFrontRight = kSteerPFrontRight;
        this.kSteerIFrontRight = kSteerIFrontRight;

        this.kDriveBackLeft = kDriveBackLeft;
        this.kSteerBackLeft = kSteerBackLeft;
        this.kEncoderBackLeft = kEncoderBackLeft;
        this.kSteerOffsetBackLeft = kSteerOffsetBackLeft;
        this.kDriveKSBackLeft = kDriveKSBackLeft;
        this.kDriveKVBackLeft = kDriveKVBackLeft;
        this.kDrivePBackLeft = kDrivePBackLeft;
        this.kDriveIBackLeft = kDriveIBackLeft;
        this.kDriveDBackLeft = kDriveDBackLeft;
        this.kSteerKSBackLeft = kSteerKSBackLeft;
        this.kSteerKVBackLeft = kSteerKVBackLeft;
        this.kSteerPBackLeft = kSteerPBackLeft;
        this.kSteerIBackLeft = kSteerIBackLeft;
        this.kSteerDBackLeft = kSteerDBackLeft;

        this.kDriveBackRight = kDriveBackRight;
        this.kSteerBackRight = kSteerBackRight;
        this.kEncoderBackRight = kEncoderBackRight;
        this.kSteerOffsetBackRight = kSteerOffsetBackRight;
        this.kDriveKSBackRight = kDriveKSBackRight;
        this.kDriveKVBackRight = kDriveKVBackRight;
        this.kDrivePBackRight = kDrivePBackRight;
        this.kDriveIBackRight = kDriveIBackRight;
        this.kDriveDBackRight = kDriveDBackRight;
        this.kSteerKSBackRight = kSteerKSBackRight;
        this.kSteerKVBackRight = kSteerKVBackRight;
        this.kSteerPBackRight = kSteerPBackRight;
        this.kSteerIBackRight = kSteerIBackRight;
        this.kSteerDBackRight = kSteerDBackRight;

        this.kDrivePAll = kDrivePAll;
        this.kDriveIAll = kDriveIAll;
        this.kDriveDAll = kDriveDAll;

        this.kSteerPAll = kSteerPAll;
        this.kSteerIAll = kSteerIAll;
        this.kSteerDAll = kSteerDAll;

        this.kDriveKSAll = kDriveKSAll;
        this.kDriveKVAll = kDriveKVAll;

        this.kSteerKS = kSteerKS;
        this.kSteerKV = kSteerKV;

        this.KheadingP = KheadingP;
        this.KheadingI = KheadingI;
        this.KheadingD = KheadingD;

        this.kDriveMotorCAN = kDriveMotorCAN;
        this.kSteerMotorCAN = kSteerMotorCAN;
        this.kPigeonCAN = kPigeonCan;
        this.kEncoderCAN = kEncoderCAN;
    }

}
