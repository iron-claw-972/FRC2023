package frc.robot.constants;

public enum ModuleConstants {
    
    FRONTLEFT(
            Constants.drive.kDriveFrontLeft,
            Constants.drive.kSteerFrontLeft,
            Constants.drive.kEncoderFrontLeft,
            Constants.drive.kSteerOffsetFrontLeft,
            Constants.drive.kDriveKSFrontLeft,
            Constants.drive.kDriveKVFrontLeft
        ),
    FRONTRIGHT(
            Constants.drive.kDriveFrontRight,
            Constants.drive.kSteerFrontRight,
            Constants.drive.kEncoderFrontRight,
            Constants.drive.kSteerOffsetFrontRight,
            Constants.drive.kDriveKSFrontRight,
            Constants.drive.kDriveKVFrontRight
        ),
    BACKLEFT(
            Constants.drive.kDriveBackLeft,
            Constants.drive.kSteerBackLeft,
            Constants.drive.kEncoderBackLeft,
            Constants.drive.kSteerOffsetBackLeft,
            Constants.drive.kDriveKSBackLeft,
            Constants.drive.kDriveKVBackLeft
    ),
    BACKRIGHT(
            Constants.drive.kDriveBackRight,
            Constants.drive.kSteerBackRight,
            Constants.drive.kEncoderBackRight,
            Constants.drive.kSteerOffsetBackRight,
            Constants.drive.kDriveKSBackRight,
            Constants.drive.kDriveKVBackRight
    ),
    NONE(0,0,0,0.0,0.0,0.0);

    private int m_drivePort;
    private int m_steerPort;
    private int m_encoderPort;
    private double m_steerOffset;
    private double m_driveKS;
    private double m_driveKV;
    
    ModuleConstants(
        int drivePort,
        int steerPort,
        int encoderPort,
        double steerOffset,
        double driveKS,
        double driveKV
    ){
        m_drivePort = drivePort;
        m_steerPort = steerPort;
        m_encoderPort = encoderPort;
        m_steerOffset = steerOffset;
        m_driveKS = driveKS;
        m_driveKV = driveKV;
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
}
