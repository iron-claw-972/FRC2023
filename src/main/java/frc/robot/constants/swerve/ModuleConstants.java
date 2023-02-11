package frc.robot.constants.swerve;

public enum ModuleConstants {

    FRONT_LEFT(
            DriveConstants.kConstants.kDriveFrontLeft,
            DriveConstants.kConstants.kSteerFrontLeft,
            DriveConstants.kConstants.kEncoderFrontLeft,
            DriveConstants.kConstants.kSteerOffsetFrontLeft,
            DriveConstants.kConstants.kDriveKSFrontLeft,
            DriveConstants.kConstants.kDriveKVFrontLeft,
            DriveConstants.kConstants.kDrivePFrontLeft,
            DriveConstants.kConstants.kDriveIFrontLeft,
            DriveConstants.kConstants.kDriveDFrontLeft,
            DriveConstants.kConstants.kSteerKSFrontLeft,
            DriveConstants.kConstants.kSteerKVFrontLeft,
            DriveConstants.kConstants.kSteerPFrontLeft,
            DriveConstants.kConstants.kSteerIFrontLeft,
            DriveConstants.kConstants.kSteerDFrontLeft,
            ModuleType.FRONT_LEFT
        ),
    FRONT_RIGHT(
            DriveConstants.kConstants.kDriveFrontRight,
            DriveConstants.kConstants.kSteerFrontRight,
            DriveConstants.kConstants.kEncoderFrontRight,
            DriveConstants.kConstants.kSteerOffsetFrontRight,
            DriveConstants.kConstants.kDriveKSFrontRight,
            DriveConstants.kConstants.kDriveKVFrontRight,
            DriveConstants.kConstants.kDrivePFrontRight,
            DriveConstants.kConstants.kDriveIFrontRight,
            DriveConstants.kConstants.kDriveDFrontRight,
            DriveConstants.kConstants.kSteerKSFrontRight,
            DriveConstants.kConstants.kSteerKVFrontRight,
            DriveConstants.kConstants.kSteerPFrontRight,
            DriveConstants.kConstants.kSteerIFrontRight,
            DriveConstants.kConstants.kSteerDFrontRight,
            ModuleType.FRONT_RIGHT
        ),
    BACK_LEFT(
            DriveConstants.kConstants.kDriveBackLeft,
            DriveConstants.kConstants.kSteerBackLeft,
            DriveConstants.kConstants.kEncoderBackLeft,
            DriveConstants.kConstants.kSteerOffsetBackLeft,
            DriveConstants.kConstants.kDriveKSBackLeft,
            DriveConstants.kConstants.kDriveKVBackLeft,
            DriveConstants.kConstants.kDrivePBackLeft,
            DriveConstants.kConstants.kDriveIBackLeft,
            DriveConstants.kConstants.kDriveDBackLeft,
            DriveConstants.kConstants.kSteerKSBackLeft,
            DriveConstants.kConstants.kSteerKVBackLeft,
            DriveConstants.kConstants.kSteerPBackLeft,
            DriveConstants.kConstants.kSteerIBackLeft,
            DriveConstants.kConstants.kSteerDBackLeft,
            ModuleType.BACK_LEFT
    ),
    BACK_RIGHT(
            DriveConstants.kConstants.kDriveBackRight,
            DriveConstants.kConstants.kSteerBackRight,
            DriveConstants.kConstants.kEncoderBackRight,
            DriveConstants.kConstants.kSteerOffsetBackRight,
            DriveConstants.kConstants.kDriveKSBackRight,
            DriveConstants.kConstants.kDriveKVBackRight,
            DriveConstants.kConstants.kDrivePBackRight,
            DriveConstants.kConstants.kDriveIBackRight,
            DriveConstants.kConstants.kDriveDBackRight,
            DriveConstants.kConstants.kSteerKSBackRight,
            DriveConstants.kConstants.kSteerKVBackRight,
            DriveConstants.kConstants.kSteerPBackRight,
            DriveConstants.kConstants.kSteerIBackRight,
            DriveConstants.kConstants.kSteerDBackRight,
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
