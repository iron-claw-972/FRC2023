package frc.robot.constants;

import frc.robot.constants.DriveConstants.CompDriveConstants;
import frc.robot.constants.DriveConstants.TestDriveConstants;

/**
 * Container class for module constants, defined using constants from {@link DriveConstants}.
 * @see DriveConstants
 */
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
    TestDriveConstants.kDriveDFrontLeft,
    TestDriveConstants.kSteerKSFrontLeft,
    TestDriveConstants.kSteerKVFrontLeft,
    TestDriveConstants.kSteerPFrontLeft,
    TestDriveConstants.kSteerIFrontLeft,
    TestDriveConstants.kSteerDFrontLeft,
    ModuleType.FRONT_LEFT
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
    TestDriveConstants.kDriveDFrontRight,
    TestDriveConstants.kSteerKSFrontRight,
    TestDriveConstants.kSteerKVFrontRight,
    TestDriveConstants.kSteerPFrontRight,
    TestDriveConstants.kSteerIFrontRight,
    TestDriveConstants.kSteerDFrontRight,
    ModuleType.FRONT_RIGHT
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
    TestDriveConstants.kDriveDBackLeft,
    TestDriveConstants.kSteerKSBackLeft,
    TestDriveConstants.kSteerKVBackLeft,
    TestDriveConstants.kSteerPBackLeft,
    TestDriveConstants.kSteerIBackLeft,
    TestDriveConstants.kSteerDBackLeft,
    ModuleType.BACK_LEFT
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
    TestDriveConstants.kDriveDBackRight,
    TestDriveConstants.kSteerKSBackRight,
    TestDriveConstants.kSteerKVBackRight,
    TestDriveConstants.kSteerPBackRight,
    TestDriveConstants.kSteerIBackRight,
    TestDriveConstants.kSteerDBackRight,
    ModuleType.BACK_RIGHT
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
    CompDriveConstants.kDriveDFrontLeft,
    CompDriveConstants.kSteerKSFrontLeft,
    CompDriveConstants.kSteerKVFrontLeft,
    CompDriveConstants.kSteerPFrontLeft,
    CompDriveConstants.kSteerIFrontLeft,
    CompDriveConstants.kSteerDFrontLeft,
    ModuleType.FRONT_LEFT
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
    CompDriveConstants.kDriveDFrontRight,
    CompDriveConstants.kSteerKSFrontRight,
    CompDriveConstants.kSteerKVFrontRight,
    CompDriveConstants.kSteerPFrontRight,
    CompDriveConstants.kSteerIFrontRight,
    CompDriveConstants.kSteerDFrontRight,
    ModuleType.FRONT_RIGHT
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
    CompDriveConstants.kDriveDBackLeft,
    CompDriveConstants.kSteerKSBackLeft,
    CompDriveConstants.kSteerKVBackLeft,
    CompDriveConstants.kSteerPBackLeft,
    CompDriveConstants.kSteerIBackLeft,
    CompDriveConstants.kSteerDBackLeft,
    ModuleType.BACK_LEFT
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
    CompDriveConstants.kDriveDBackRight,
    CompDriveConstants.kSteerKSBackRight,
    CompDriveConstants.kSteerKVBackRight,
    CompDriveConstants.kSteerPBackRight,
    CompDriveConstants.kSteerIBackRight,
    CompDriveConstants.kSteerDBackRight,
    ModuleType.BACK_RIGHT
  ),
  NONE(0,0,0,0.0,0.0,0.0,0.0,0.0,0.0,0,0,0,0,0, ModuleType.NONE);
  
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
  ) {
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
  public double getDriveP() {
    return m_driveP;
  }
  public double getDriveI() {
    return m_driveI;
  }
  public double getDriveD() {
    return m_driveD;
  }
  public double getSteerKS() {
    return m_steerKS;
  }
  public double getSteerKV() {
    return m_steerKV;
  }
  public double getSteerP() {
    return m_steerP;
  }
  public double getSteerI() {
    return m_steerI;
  }
  public double getSteerD() {
    return m_steerD;
  }
  public ModuleType getType() {
    return m_type;
  }
}
