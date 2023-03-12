package frc.robot.constants.swerve;

/**
 * Container class for module constants, defined using constants from {@link DriveConstants}
 * .
 * @see DriveConstants
 */
public enum ModuleConstants {
  
  FRONT_LEFT(
    DriveConstants.kDriveFrontLeft,
    DriveConstants.kSteerFrontLeft,
    DriveConstants.kEncoderFrontLeft,
    DriveConstants.kSteerOffsetFrontLeft,
    ModuleType.FRONT_LEFT
  ),
  FRONT_RIGHT(
    DriveConstants.kDriveFrontRight,
    DriveConstants.kSteerFrontRight,
    DriveConstants.kEncoderFrontRight,
    DriveConstants.kSteerOffsetFrontRight,
    ModuleType.FRONT_RIGHT
  ),
  BACK_LEFT(
    DriveConstants.kDriveBackLeft,
    DriveConstants.kSteerBackLeft,
    DriveConstants.kEncoderBackLeft,
    DriveConstants.kSteerOffsetBackLeft,
    ModuleType.BACK_LEFT
  ),
  BACK_RIGHT(
    DriveConstants.kDriveBackRight,
    DriveConstants.kSteerBackRight,
    DriveConstants.kEncoderBackRight,
    DriveConstants.kSteerOffsetBackRight,
    ModuleType.BACK_RIGHT
  ),

  NONE(0, 0, 0, 0.0, ModuleType.NONE);
  
  private final int m_drivePort;
  private final int m_steerPort;
  private final int m_encoderPort;
  private final double m_steerOffset;
  private final ModuleType m_type;
  
  ModuleConstants(
    int drivePort,
    int steerPort,
    int encoderPort,
    double steerOffset,
    ModuleType type
  ) {
    m_drivePort = drivePort;
    m_steerPort = steerPort;
    m_encoderPort = encoderPort;
    m_steerOffset = steerOffset;
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

  public ModuleType getType() {
    return m_type;
  }

}