package frc.robot.constants.swerve;

import frc.robot.constants.Constants;

public enum ModuleConstants {

    FRONT_LEFT(
            Constants.drive.kDriveFrontLeft,
            Constants.drive.kSteerFrontLeft,
            Constants.drive.kEncoderFrontLeft,
            Constants.drive.kSteerOffsetFrontLeft,
            Constants.drive.kDriveKSFrontLeft,
            Constants.drive.kDriveKVFrontLeft,
            Constants.drive.kDrivePFrontLeft,
            Constants.drive.kDriveIFrontLeft,
            Constants.drive.kDriveDFrontLeft,
            Constants.drive.kSteerKSFrontLeft,
            Constants.drive.kSteerKVFrontLeft,
            Constants.drive.kSteerPFrontLeft,
            Constants.drive.kSteerIFrontLeft,
            Constants.drive.kSteerDFrontLeft
        ),
    FRONT_RIGHT(
            Constants.drive.kDriveFrontRight,
            Constants.drive.kSteerFrontRight,
            Constants.drive.kEncoderFrontRight,
            Constants.drive.kSteerOffsetFrontRight,
            Constants.drive.kDriveKSFrontRight,
            Constants.drive.kDriveKVFrontRight,
            Constants.drive.kDrivePFrontRight,
            Constants.drive.kDriveIFrontRight,
            Constants.drive.kDriveDFrontRight,
            Constants.drive.kSteerKSFrontRight,
            Constants.drive.kSteerKVFrontRight,
            Constants.drive.kSteerPFrontRight,
            Constants.drive.kSteerIFrontRight,
            Constants.drive.kSteerDFrontRight
        ),
    BACK_LEFT(
            Constants.drive.kDriveBackLeft,
            Constants.drive.kSteerBackLeft,
            Constants.drive.kEncoderBackLeft,
            Constants.drive.kSteerOffsetBackLeft,
            Constants.drive.kDriveKSBackLeft,
            Constants.drive.kDriveKVBackLeft,
            Constants.drive.kDrivePBackLeft,
            Constants.drive.kDriveIBackLeft,
            Constants.drive.kDriveDBackLeft,
            Constants.drive.kSteerKSBackLeft,
            Constants.drive.kSteerKVBackLeft,
            Constants.drive.kSteerPBackLeft,
            Constants.drive.kSteerIBackLeft,
            Constants.drive.kSteerDBackLeft
    ),
    BACK_RIGHT(
            Constants.drive.kDriveBackRight,
            Constants.drive.kSteerBackRight,
            Constants.drive.kEncoderBackRight,
            Constants.drive.kSteerOffsetBackRight,
            Constants.drive.kDriveKSBackRight,
            Constants.drive.kDriveKVBackRight,
            Constants.drive.kDrivePBackRight,
            Constants.drive.kDriveIBackRight,
            Constants.drive.kDriveDBackRight,
            Constants.drive.kSteerKSBackRight,
            Constants.drive.kSteerKVBackRight,
            Constants.drive.kSteerPBackRight,
            Constants.drive.kSteerIBackRight,
            Constants.drive.kSteerDBackRight
    ),
    NONE(0,0,0,0.0,0.0,0.0,0.0,0.0,0.0,0,0,0,0,0);

    private int m_drivePort;
    private int m_steerPort;
    private int m_encoderPort;
    private double m_steerOffset;
    private double m_driveKS;
    private double m_driveKV;
    private double m_driveP;
    private double m_driveI;
    private double m_driveD;
    private double m_steerKS;
    private double m_steerKV;
    private double m_steerP;
    private double m_steerI;
    private double m_steerD;
    
    ModuleConstants(
        int drivePort,
        int steerPort,
        int encoderPort,
        double steerOffset,
        double driveKS,
        double driveKV,
        double driveP,
        double driveI,
        double driveD,
        double steerKS,
        double steerKV,
        double steerP,
        double steerI,
        double steerD
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
        m_steerKS = steerKS;
        m_steerKV = steerKV;
        m_steerP = steerP;
        m_steerI = steerI;
        m_steerD = steerD;


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

    public double getSteerKS() {
        return m_steerKS;
    }
    public double getSteerKV() {
        return m_steerKV;
    }

    public double getSteerP(){
        return m_steerP;
    }
    public double getSteerI(){
        return m_steerI;
    }
    public double getSteerD(){
        return m_steerD;
    }
}
