package frc.robot.constants;

import frc.robot.constants.DriveConstants.CompDriveConstants;
import frc.robot.constants.DriveConstants.TestDriveConstants;

public enum ModuleConstants {
    
    TEST_FL(
            TestDriveConstants.kDriveFrontLeft,
            TestDriveConstants.kSteerFrontLeft,
            TestDriveConstants.kEncoderFrontLeft,
            TestDriveConstants.kSteerOffsetFrontLeft,
            TestDriveConstants.kDriveKSFrontLeft,
            TestDriveConstants.kDriveKVFrontLeft,
            TestDriveConstants.kDrivePFrontLeft,
            TestDriveConstants.kDriveIFrontLeft,
            TestDriveConstants.kDriveDFrontLeft
        ),
    TEST_FR(
            TestDriveConstants.kDriveFrontRight,
            TestDriveConstants.kSteerFrontRight,
            TestDriveConstants.kEncoderFrontRight,
            TestDriveConstants.kSteerOffsetFrontRight,
            TestDriveConstants.kDriveKSFrontRight,
            TestDriveConstants.kDriveKVFrontRight,
            TestDriveConstants.kDrivePFrontRight,
            TestDriveConstants.kDriveIFrontRight,
            TestDriveConstants.kDriveDFrontRight
        ),
    TEST_BL(
            TestDriveConstants.kDriveBackLeft,
            TestDriveConstants.kSteerBackLeft,
            TestDriveConstants.kEncoderBackLeft,
            TestDriveConstants.kSteerOffsetBackLeft,
            TestDriveConstants.kDriveKSBackLeft,
            TestDriveConstants.kDriveKVBackLeft,
            TestDriveConstants.kDrivePBackLeft,
            TestDriveConstants.kDriveIBackLeft,
            TestDriveConstants.kDriveDBackLeft
    ),
    TEST_BR(
            TestDriveConstants.kDriveBackRight,
            TestDriveConstants.kSteerBackRight,
            TestDriveConstants.kEncoderBackRight,
            TestDriveConstants.kSteerOffsetBackRight,
            TestDriveConstants.kDriveKSBackRight,
            TestDriveConstants.kDriveKVBackRight,
            TestDriveConstants.kDrivePBackRight,
            TestDriveConstants.kDriveIBackRight,
            TestDriveConstants.kDriveDBackRight
    ),

    COMP_FL(
            CompDriveConstants.kDriveFrontLeft,
            CompDriveConstants.kSteerFrontLeft,
            CompDriveConstants.kEncoderFrontLeft,
            CompDriveConstants.kSteerOffsetFrontLeft,
            CompDriveConstants.kDriveKSFrontLeft,
            CompDriveConstants.kDriveKVFrontLeft,
            CompDriveConstants.kDrivePFrontLeft,
            CompDriveConstants.kDriveIFrontLeft,
            CompDriveConstants.kDriveDFrontLeft
        ),
    COMP_FR(
            CompDriveConstants.kDriveFrontRight,
            CompDriveConstants.kSteerFrontRight,
            CompDriveConstants.kEncoderFrontRight,
            CompDriveConstants.kSteerOffsetFrontRight,
            CompDriveConstants.kDriveKSFrontRight,
            CompDriveConstants.kDriveKVFrontRight,
            CompDriveConstants.kDrivePFrontRight,
            CompDriveConstants.kDriveIFrontRight,
            CompDriveConstants.kDriveDFrontRight
        ),
    COMP_BL(
            CompDriveConstants.kDriveBackLeft,
            CompDriveConstants.kSteerBackLeft,
            CompDriveConstants.kEncoderBackLeft,
            CompDriveConstants.kSteerOffsetBackLeft,
            CompDriveConstants.kDriveKSBackLeft,
            CompDriveConstants.kDriveKVBackLeft,
            CompDriveConstants.kDrivePBackLeft,
            CompDriveConstants.kDriveIBackLeft,
            CompDriveConstants.kDriveDBackLeft
    ),
    COMP_BR(
            CompDriveConstants.kDriveBackRight,
            CompDriveConstants.kSteerBackRight,
            CompDriveConstants.kEncoderBackRight,
            CompDriveConstants.kSteerOffsetBackRight,
            CompDriveConstants.kDriveKSBackRight,
            CompDriveConstants.kDriveKVBackRight,
            CompDriveConstants.kDrivePBackRight,
            CompDriveConstants.kDriveIBackRight,
            CompDriveConstants.kDriveDBackRight
    ),
    NONE(0,0,0,0.0,0.0,0.0,0.0,0.0,0.0);

    private int m_drivePort;
    private int m_steerPort;
    private int m_encoderPort;
    private double m_steerOffset;
    private double m_driveKS;
    private double m_driveKV;
    public final double m_driveP;
    public final double m_driveI;
    public final double m_driveD;
    
    ModuleConstants(
        int drivePort,
        int steerPort,
        int encoderPort,
        double steerOffset,
        double driveKS,
        double driveKV,
        double driveP,
        double driveI,
        double driveD
    ){
        m_drivePort = drivePort;
        m_steerPort = steerPort;
        m_encoderPort = encoderPort;
        m_steerOffset = steerOffset;
        m_driveKS = driveKS;
        m_driveKV = driveKV;
        m_driveP = driveP;
        m_driveI = driveI;
        m_driveD = driveD;

    }
    public int getDrivePort() {
        return m_drivePort;
    }
    public int getSteerPort() {
        return m_steerPort;
    }
    public int getEncoderPort() {
        return m_encoderPort;
    }
    public double getSteerOffset() {
        return m_steerOffset;
    }
    public double getDriveKS() {
        return m_driveKS;
    }
    public double getDriveKV() {
        return m_driveKV;
    }
    public double getDriveP(){
        return m_driveP;
    }
    public double getDriveI(){
        return m_driveI;
    }
    public double getDriveD(){
        return m_driveD;
    }
}
