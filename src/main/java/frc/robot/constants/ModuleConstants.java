package frc.robot.constants;

import frc.robot.constants.Constants;
import frc.robot.constants.swerve.CompDriveConstants;

public enum ModuleConstants {

    FRONT_LEFT(
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
            CompDriveConstants.kSteerDFrontLeft
        ),
    FORNT_RIGHT(
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
            CompDriveConstants.kSteerDFrontRight
        ),
    BACK_LEFT(
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
            CompDriveConstants.kSteerDBackLeft
    ),
    BACK_RIGHT(
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
            CompDriveConstants.kSteerDBackRight
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
