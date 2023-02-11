package frc.robot.constants.swerve;

public enum ModuleConstants {

    FRONT_LEFT(
            DriveConstants.kDriveFrontLeft,
            DriveConstants.kSteerFrontLeft,
            DriveConstants.kEncoderFrontLeft,
            DriveConstants.kSteerOffsetFrontLeft,
            DriveConstants.kDriveKSFrontLeft,
            DriveConstants.kDriveKVFrontLeft,
            DriveConstants.kDrivePFrontLeft,
            DriveConstants.kDriveIFrontLeft,
            DriveConstants.kDriveDFrontLeft,
            DriveConstants.kSteerKSFrontLeft,
            DriveConstants.kSteerKVFrontLeft,
            DriveConstants.kSteerPFrontLeft,
            DriveConstants.kSteerIFrontLeft,
            DriveConstants.kSteerDFrontLeft,
            ModuleType.FRONT_LEFT
        ),
    FRONT_RIGHT(
            DriveConstants.kDriveFrontRight,
            DriveConstants.kSteerFrontRight,
            DriveConstants.kEncoderFrontRight,
            DriveConstants.kSteerOffsetFrontRight,
            DriveConstants.kDriveKSFrontRight,
            DriveConstants.kDriveKVFrontRight,
            DriveConstants.kDrivePFrontRight,
            DriveConstants.kDriveIFrontRight,
            DriveConstants.kDriveDFrontRight,
            DriveConstants.kSteerKSFrontRight,
            DriveConstants.kSteerKVFrontRight,
            DriveConstants.kSteerPFrontRight,
            DriveConstants.kSteerIFrontRight,
            DriveConstants.kSteerDFrontRight,
            ModuleType.FRONT_RIGHT
        ),
    BACK_LEFT(
            DriveConstants.kDriveBackLeft,
            DriveConstants.kSteerBackLeft,
            DriveConstants.kEncoderBackLeft,
            DriveConstants.kSteerOffsetBackLeft,
            DriveConstants.kDriveKSBackLeft,
            DriveConstants.kDriveKVBackLeft,
            DriveConstants.kDrivePBackLeft,
            DriveConstants.kDriveIBackLeft,
            DriveConstants.kDriveDBackLeft,
            DriveConstants.kSteerKSBackLeft,
            DriveConstants.kSteerKVBackLeft,
            DriveConstants.kSteerPBackLeft,
            DriveConstants.kSteerIBackLeft,
            DriveConstants.kSteerDBackLeft,
            ModuleType.BACK_LEFT
    ),
    BACK_RIGHT(
            DriveConstants.kDriveBackRight,
            DriveConstants.kSteerBackRight,
            DriveConstants.kEncoderBackRight,
            DriveConstants.kSteerOffsetBackRight,
            DriveConstants.kDriveKSBackRight,
            DriveConstants.kDriveKVBackRight,
            DriveConstants.kDrivePBackRight,
            DriveConstants.kDriveIBackRight,
            DriveConstants.kDriveDBackRight,
            DriveConstants.kSteerKSBackRight,
            DriveConstants.kSteerKVBackRight,
            DriveConstants.kSteerPBackRight,
            DriveConstants.kSteerIBackRight,
            DriveConstants.kSteerDBackRight,
            ModuleType.BACK_RIGHT
    ),
    NONE(0,0,0,0.0,0.0,0.0,0.0,0.0,0.0,0,0,0,0,0,ModuleType.NONE);

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
    private ModuleType m_type;
    
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
        double steerD,
        ModuleType type
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
        m_type = type;
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
    public ModuleType getType(){
        return m_type;
    }
}
