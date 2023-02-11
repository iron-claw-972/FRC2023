package frc.robot.constants.swerve;

public enum ModuleConstants {

    FRONT_LEFT(
            DriveConstants.kConstants.kConstants.kDriveFrontLeft,
            DriveConstants.kConstants.kConstants.kSteerFrontLeft,
            DriveConstants.kConstants.kConstants.kEncoderFrontLeft,
            DriveConstants.kConstants.kConstants.kSteerOffsetFrontLeft,
            DriveConstants.kConstants.kConstants.kDriveKSFrontLeft,
            DriveConstants.kConstants.kConstants.kDriveKVFrontLeft,
            DriveConstants.kConstants.kConstants.kDrivePFrontLeft,
            DriveConstants.kConstants.kConstants.kDriveIFrontLeft,
            DriveConstants.kConstants.kConstants.kDriveDFrontLeft,
            DriveConstants.kConstants.kConstants.kSteerKSFrontLeft,
            DriveConstants.kConstants.kConstants.kSteerKVFrontLeft,
            DriveConstants.kConstants.kConstants.kSteerPFrontLeft,
            DriveConstants.kConstants.kConstants.kSteerIFrontLeft,
            DriveConstants.kConstants.kConstants.kSteerDFrontLeft,
            ModuleType.FRONT_LEFT
        ),
    FRONT_RIGHT(
            DriveConstants.kConstants.kConstants.kDriveFrontRight,
            DriveConstants.kConstants.kConstants.kSteerFrontRight,
            DriveConstants.kConstants.kConstants.kEncoderFrontRight,
            DriveConstants.kConstants.kConstants.kSteerOffsetFrontRight,
            DriveConstants.kConstants.kConstants.kDriveKSFrontRight,
            DriveConstants.kConstants.kConstants.kDriveKVFrontRight,
            DriveConstants.kConstants.kConstants.kDrivePFrontRight,
            DriveConstants.kConstants.kConstants.kDriveIFrontRight,
            DriveConstants.kConstants.kConstants.kDriveDFrontRight,
            DriveConstants.kConstants.kConstants.kSteerKSFrontRight,
            DriveConstants.kConstants.kConstants.kSteerKVFrontRight,
            DriveConstants.kConstants.kConstants.kSteerPFrontRight,
            DriveConstants.kConstants.kConstants.kSteerIFrontRight,
            DriveConstants.kConstants.kConstants.kSteerDFrontRight,
            ModuleType.FRONT_RIGHT
        ),
    BACK_LEFT(
            DriveConstants.kConstants.kConstants.kDriveBackLeft,
            DriveConstants.kConstants.kConstants.kSteerBackLeft,
            DriveConstants.kConstants.kConstants.kEncoderBackLeft,
            DriveConstants.kConstants.kConstants.kSteerOffsetBackLeft,
            DriveConstants.kConstants.kConstants.kDriveKSBackLeft,
            DriveConstants.kConstants.kConstants.kDriveKVBackLeft,
            DriveConstants.kConstants.kConstants.kDrivePBackLeft,
            DriveConstants.kConstants.kConstants.kDriveIBackLeft,
            DriveConstants.kConstants.kConstants.kDriveDBackLeft,
            DriveConstants.kConstants.kConstants.kSteerKSBackLeft,
            DriveConstants.kConstants.kConstants.kSteerKVBackLeft,
            DriveConstants.kConstants.kConstants.kSteerPBackLeft,
            DriveConstants.kConstants.kConstants.kSteerIBackLeft,
            DriveConstants.kConstants.kConstants.kSteerDBackLeft,
            ModuleType.BACK_LEFT
    ),
    BACK_RIGHT(
            DriveConstants.kConstants.kConstants.kDriveBackRight,
            DriveConstants.kConstants.kConstants.kSteerBackRight,
            DriveConstants.kConstants.kConstants.kEncoderBackRight,
            DriveConstants.kConstants.kConstants.kSteerOffsetBackRight,
            DriveConstants.kConstants.kConstants.kDriveKSBackRight,
            DriveConstants.kConstants.kConstants.kDriveKVBackRight,
            DriveConstants.kConstants.kConstants.kDrivePBackRight,
            DriveConstants.kConstants.kConstants.kDriveIBackRight,
            DriveConstants.kConstants.kConstants.kDriveDBackRight,
            DriveConstants.kConstants.kConstants.kSteerKSBackRight,
            DriveConstants.kConstants.kConstants.kSteerKVBackRight,
            DriveConstants.kConstants.kConstants.kSteerPBackRight,
            DriveConstants.kConstants.kConstants.kSteerIBackRight,
            DriveConstants.kConstants.kConstants.kSteerDBackRight,
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
