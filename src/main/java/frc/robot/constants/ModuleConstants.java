package frc.robot.constants;

public enum ModuleConstants {
    
    FRONTLEFT(
            Constants.drive.kDriveFrontLeft,
            Constants.drive.kSteerFrontLeft,
            Constants.drive.kEncoderFrontLeft,
            Constants.drive.kSteerOffsetFrontLeft,
            Constants.drive.kDriveKSFrontLeft,
            Constants.drive.kDriveKVFrontLeft,
            Constants.drive.kDrivePFrontLeft,
            Constants.drive.kDriveIFrontLeft,
            Constants.drive.kDriveDFrontLeft
        ),
    FRONTRIGHT(
            Constants.drive.kDriveFrontRight,
            Constants.drive.kSteerFrontRight,
            Constants.drive.kEncoderFrontRight,
            Constants.drive.kSteerOffsetFrontRight,
            Constants.drive.kDriveKSFrontRight,
            Constants.drive.kDriveKVFrontRight,
            Constants.drive.kDrivePFrontRight,
            Constants.drive.kDriveIFrontRight,
            Constants.drive.kDriveDFrontRight
        ),
    BACKLEFT(
            Constants.drive.kDriveBackLeft,
            Constants.drive.kSteerBackLeft,
            Constants.drive.kEncoderBackLeft,
            Constants.drive.kSteerOffsetBackLeft,
            Constants.drive.kDriveKSBackLeft,
            Constants.drive.kDriveKVBackLeft,
            Constants.drive.kDrivePBackLeft,
            Constants.drive.kDriveIBackLeft,
            Constants.drive.kDriveDBackLeft
    ),
    BACKRIGHT(
            Constants.drive.kDriveBackRight,
            Constants.drive.kSteerBackRight,
            Constants.drive.kEncoderBackRight,
            Constants.drive.kSteerOffsetBackRight,
            Constants.drive.kDriveKSBackRight,
            Constants.drive.kDriveKVBackRight,
            Constants.drive.kDrivePBackRight,
            Constants.drive.kDriveIBackRight,
            Constants.drive.kDriveDBackRight
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
