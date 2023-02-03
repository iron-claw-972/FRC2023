package frc.robot.constants.swerve;

public enum SwerveDriveConstants {

    COMP(
        CompDriveConstants.kWheelRadius,

        CompDriveConstants.kTrackWidth,
        CompDriveConstants.kDriveGearRatio,
        CompDriveConstants.kSteerGearRatio,

        CompDriveConstants.kMaxSpeed,

        CompDriveConstants.kMaxAngularSpeed,
        CompDriveConstants.kMaxAngularAccel,

        CompDriveConstants.kPigeon,

        CompDriveConstants.kStartingHeadingDegrees,

        CompDriveConstants.kDriveFrontLeft,
        CompDriveConstants.kSteerFrontLeft,
        CompDriveConstants.kEncoderFrontLeft,
        CompDriveConstants.kSteerOffsetFrontLeft,
        CompDriveConstants.kDriveKSFrontLeft,
        CompDriveConstants.kDriveKVFrontLeft,
        CompDriveConstants.kDrivePFrontLeft,
        CompDriveConstants.kDriveIFrontLeft,
        CompDriveConstants.kDriveDFrontLeft,
        CompDriveConstants.kSteerKSFrontLeft,
        CompDriveConstants.kSteerKVFrontLeft,
        CompDriveConstants.kSteerPFrontLeft,
        CompDriveConstants.kSteerIFrontLeft,
        CompDriveConstants.kSteerDFrontLeft,

        CompDriveConstants.kDriveFrontRight, 
        CompDriveConstants.kSteerFrontRight,
        CompDriveConstants.kEncoderFrontRight, 
        CompDriveConstants.kSteerOffsetFrontRight,
        CompDriveConstants.kDriveKSFrontRight,
        CompDriveConstants.kDriveKVFrontRight,
        CompDriveConstants.kDrivePFrontRight,
        CompDriveConstants.kDriveIFrontRight,
        CompDriveConstants.kDriveDFrontRight,
        CompDriveConstants.kSteerKSFrontRight,
        CompDriveConstants.kSteerKVFrontRight,
        CompDriveConstants.kSteerPFrontRight,
        CompDriveConstants.kSteerIFrontRight,
        CompDriveConstants.kSteerDFrontRight,

        CompDriveConstants.kDriveBackLeft,
        CompDriveConstants.kSteerBackLeft,
        CompDriveConstants.kEncoderBackLeft, 
        CompDriveConstants.kSteerOffsetBackLeft,
        CompDriveConstants.kDriveKSBackLeft,
        CompDriveConstants.kDriveKVBackLeft,
        CompDriveConstants.kDrivePBackLeft,
        CompDriveConstants.kDriveIBackLeft,
        CompDriveConstants.kDriveDBackLeft,
        CompDriveConstants.kSteerKSBackLeft,
        CompDriveConstants.kSteerKVBackLeft,
        CompDriveConstants.kSteerPBackLeft,
        CompDriveConstants.kSteerIBackLeft,
        CompDriveConstants.kSteerDBackLeft,

        CompDriveConstants.kDriveBackRight,
        CompDriveConstants.kSteerBackRight,
        CompDriveConstants.kEncoderBackRight, 
        CompDriveConstants.kSteerOffsetBackRight,
        CompDriveConstants.kDriveKSBackRight,
        CompDriveConstants.kDriveKVBackRight,
        CompDriveConstants.kDrivePBackRight,
        CompDriveConstants.kDriveIBackRight,
        CompDriveConstants.kDriveDBackRight,
        CompDriveConstants.kSteerKSBackRight,
        CompDriveConstants.kSteerKVBackRight,
        CompDriveConstants.kSteerPBackRight,
        CompDriveConstants.kSteerIBackRight,
        CompDriveConstants.kSteerDBackRight,

        // Drive
        CompDriveConstants.kDrivePAll,
        CompDriveConstants.kDriveIAll,
        CompDriveConstants.kDriveDAll,
        
        // Steer
        CompDriveConstants.kSteerPAll, //SDS: 0.2 - maunfacturer recomended values
        CompDriveConstants.kSteerIAll,
        CompDriveConstants.kSteerDAll, //SDS: 0.1 - maunfacturer recomended values

        CompDriveConstants.kDriveKSAll,
        CompDriveConstants.kDriveKVAll,

        // Steer
        CompDriveConstants.kSteerKS,
        CompDriveConstants.kSteerKV,

        // heading PID
        CompDriveConstants.KheadingP,
        CompDriveConstants.KheadingI,
        CompDriveConstants.KheadingD
    ),

    TEST(
        TestDriveConstants.kWheelRadius,

        TestDriveConstants.kTrackWidth,
        TestDriveConstants.kDriveGearRatio,
        TestDriveConstants.kSteerGearRatio,

        TestDriveConstants.kMaxSpeed,

        TestDriveConstants.kMaxAngularSpeed,
        TestDriveConstants.kMaxAngularAccel,

        TestDriveConstants.kPigeon,

        TestDriveConstants.kStartingHeadingDegrees,

        TestDriveConstants.kDriveFrontLeft,
        TestDriveConstants.kSteerFrontLeft,
        TestDriveConstants.kEncoderFrontLeft,
        TestDriveConstants.kSteerOffsetFrontLeft,
        TestDriveConstants.kDriveKSFrontLeft,
        TestDriveConstants.kDriveKVFrontLeft,
        TestDriveConstants.kDrivePFrontLeft,
        TestDriveConstants.kDriveIFrontLeft,
        TestDriveConstants.kDriveDFrontLeft,
        TestDriveConstants.kSteerKSFrontLeft,
        TestDriveConstants.kSteerKVFrontLeft,
        TestDriveConstants.kSteerPFrontLeft,
        TestDriveConstants.kSteerIFrontLeft,
        TestDriveConstants.kSteerDFrontLeft,

        TestDriveConstants.kDriveFrontRight, 
        TestDriveConstants.kSteerFrontRight,
        TestDriveConstants.kEncoderFrontRight, 
        TestDriveConstants.kSteerOffsetFrontRight,
        TestDriveConstants.kDriveKSFrontRight,
        TestDriveConstants.kDriveKVFrontRight,
        TestDriveConstants.kDrivePFrontRight,
        TestDriveConstants.kDriveIFrontRight,
        TestDriveConstants.kDriveDFrontRight,
        TestDriveConstants.kSteerKSFrontRight,
        TestDriveConstants.kSteerKVFrontRight,
        TestDriveConstants.kSteerPFrontRight,
        TestDriveConstants.kSteerIFrontRight,
        TestDriveConstants.kSteerDFrontRight,

        TestDriveConstants.kDriveBackLeft,
        TestDriveConstants.kSteerBackLeft,
        TestDriveConstants.kEncoderBackLeft, 
        TestDriveConstants.kSteerOffsetBackLeft,
        TestDriveConstants.kDriveKSBackLeft,
        TestDriveConstants.kDriveKVBackLeft,
        TestDriveConstants.kDrivePBackLeft,
        TestDriveConstants.kDriveIBackLeft,
        TestDriveConstants.kDriveDBackLeft,
        TestDriveConstants.kSteerKSBackLeft,
        TestDriveConstants.kSteerKVBackLeft,
        TestDriveConstants.kSteerPBackLeft,
        TestDriveConstants.kSteerIBackLeft,
        TestDriveConstants.kSteerDBackLeft,

        TestDriveConstants.kDriveBackRight,
        TestDriveConstants.kSteerBackRight,
        TestDriveConstants.kEncoderBackRight, 
        TestDriveConstants.kSteerOffsetBackRight,
        TestDriveConstants.kDriveKSBackRight,
        TestDriveConstants.kDriveKVBackRight,
        TestDriveConstants.kDrivePBackRight,
        TestDriveConstants.kDriveIBackRight,
        TestDriveConstants.kDriveDBackRight,
        TestDriveConstants.kSteerKSBackRight,
        TestDriveConstants.kSteerKVBackRight,
        TestDriveConstants.kSteerPBackRight,
        TestDriveConstants.kSteerIBackRight,
        TestDriveConstants.kSteerDBackRight,

        // Drive
        TestDriveConstants.kDrivePAll,
        TestDriveConstants.kDriveIAll,
        TestDriveConstants.kDriveDAll,
        
        // Steer
        TestDriveConstants.kSteerPAll, //SDS: 0.2 - maunfacturer recomended values
        TestDriveConstants.kSteerIAll,
        TestDriveConstants.kSteerDAll, //SDS: 0.1 - maunfacturer recomended values

        TestDriveConstants.kDriveKSAll,
        TestDriveConstants.kDriveKVAll,

        // Steer
        TestDriveConstants.kSteerKS,
        TestDriveConstants.kSteerKV,

        // heading PID
        TestDriveConstants.KheadingP,
        TestDriveConstants.KheadingI,
        TestDriveConstants.KheadingD
    );

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

    SwerveDriveConstants(
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
        double KheadingD
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
    }

}
