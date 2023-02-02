package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public abstract class SwerveDriveConstants {

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

}
