package frc.robot.subsystems;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import lib.CTREModuleState;
import frc.robot.constants.Constants;
import frc.robot.constants.swerve.DriveConstants;
import frc.robot.constants.swerve.ModuleConstants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.ctre.phoenix.sensors.WPI_CANCoder;

public class ModuleChanged {
  private final ShuffleboardTab m_swerveTab;

  private final int m_moduleIndex;
  private Rotation2d m_angleOffset;
  private Rotation2d m_lastAngle;

  private final String m_moduleAbbr;

  private final WPI_TalonFX m_angleMotor;
  private final WPI_TalonFX m_driveMotor;
  private final WPI_CANCoder m_CANcoder;
  private SwerveModuleState m_desiredState;

  private boolean m_angleJitterPreventionEnabled;

  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(DriveConstants.kDriveKS, DriveConstants.kDriveKV, DriveConstants.kDriveKA);

  public ModuleChanged(ModuleConstants moduleConstants, ShuffleboardTab swerveTab) {
    m_swerveTab = swerveTab;
    m_moduleIndex = moduleConstants.getType().id;
    m_moduleAbbr = moduleConstants.getType().abbrev;
    m_angleOffset = new Rotation2d(moduleConstants.getSteerOffset());

    m_angleJitterPreventionEnabled = true;

    /* Angle Encoder Config */
    m_CANcoder = new WPI_CANCoder(moduleConstants.getEncoderPort(), DriveConstants.kSteerEncoderCAN);
    configCANcoder();

    /* Angle Motor Config */
    m_angleMotor = new WPI_TalonFX(moduleConstants.getSteerPort(), DriveConstants.kSteerEncoderCAN);
    configAngleMotor();

    /* Drive Motor Config */
    m_driveMotor = new WPI_TalonFX(moduleConstants.getDrivePort(), DriveConstants.kDriveMotorCAN);
    configDriveMotor();

    m_lastAngle = getState().angle;
    setDesiredState(new SwerveModuleState(0, m_lastAngle), false);

    setupShuffleboard();
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    /*
     * This is a custom optimize function, since default WPILib optimize assumes
     * continuous controller which CTRE and Rev onboard is not
     */
    desiredState = CTREModuleState.optimize(desiredState, getState().angle);
    m_desiredState = desiredState;
    setAngle(desiredState);
    setSpeed(desiredState, isOpenLoop);
  }

  private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
    if (isOpenLoop) {
      double percentOutput = desiredState.speedMetersPerSecond / DriveConstants.kMaxSpeed;
      m_driveMotor.set(ControlMode.PercentOutput, percentOutput);
    } else {
      double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, DriveConstants.kWheelCircumference,
          DriveConstants.kDriveGearRatio);
      m_driveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward,
          feedforward.calculate(desiredState.speedMetersPerSecond));
    }
  }

  private void setAngle(SwerveModuleState desiredState) {
    // Prevent rotating module if desired speed < 1%. Prevents Jittering.
    Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (DriveConstants.kMaxSpeed * 0.01)
        && m_angleJitterPreventionEnabled) ? m_lastAngle : desiredState.angle;

    m_angleMotor.set(ControlMode.Position, Conversions.degreesToFalcon(angle.getDegrees(), DriveConstants.kAngleGearRatio));
    m_lastAngle = angle;
  }

  public void toggleAngleJitterPrevention(boolean enabled) {
    m_angleJitterPreventionEnabled = enabled;
  }

  public int getModuleIndex() {
    return m_moduleIndex;
  }

  private Rotation2d getAngle() {
    return Rotation2d.fromDegrees(
        Conversions.falconToDegrees(m_angleMotor.getSelectedSensorPosition(), SwerveConstants.kAngleGearRatio));
  }

  public Rotation2d getCANcoder() {
    return Rotation2d.fromDegrees(m_CANcoder.getAbsolutePosition());
  }

  public void resetToAbsolute() {
    double absolutePosition = Conversions.degreesToFalcon(getCANcoder().getDegrees() - m_angleOffset.getDegrees(),
        DriveConstants.kAngleGearRatio);
    m_angleMotor.setSelectedSensorPosition(absolutePosition);
  }

  private void configCANcoder() {
    m_CANcoder.configFactoryDefault();
    m_CANcoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
    m_CANcoder.configSensorDirection(DriveConstants.kModuleConstants.canCoderInvert);
    m_CANcoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
    m_CANcoder.configFeedbackCoefficient(0.087890625, "deg", SensorTimeBase.PerSecond);
  }

  private void configAngleMotor() {
    m_angleMotor.configFactoryDefault();
    m_angleMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(
      DriveConstants.kAngleEnableCurrentLimit,
      DriveConstants.kAngleContinuousCurrentLimit,
      DriveConstants.kAnglePeakCurrentLimit,
      DriveConstants.kAnglePeakCurrentDuration
    ));
    m_angleMotor.config_kP(0, DriveConstants.kModuleConstants.angleKP);
    m_angleMotor.config_kI(0, DriveConstants.kModuleConstants.angleKI);
    m_angleMotor.config_kD(0, DriveConstants.kModuleConstants.angleKD);
    m_angleMotor.config_kF(0, DriveConstants.kModuleConstants.angleKF);
    m_angleMotor.setInverted(DriveConstants.kAngleMotorInvert);
    m_angleMotor.setNeutralMode(DriveConstants.kAngleNeutralMode);
    m_angleMotor.configVoltageCompSaturation(Constants.kMaxVoltage);
    m_angleMotor.enableVoltageCompensation(true);
    resetToAbsolute();
  }

  public void setDriveCharacterizationVoltage(double voltage) {
    m_angleMotor.set(ControlMode.Position, Conversions.degreesToFalcon(0, DriveConstants.kAngleGearRatio));
    m_driveMotor.set(ControlMode.PercentOutput, voltage / DriveConstants.kNormalOperatingVoltage);
  }

  public void setAngleCharacterizationVoltage(double voltage) {
    m_angleMotor.set(ControlMode.PercentOutput, voltage / DriveConstants.kNormalOperatingVoltage);
    // Set the drive motor to just enough to overcome static friction
    m_driveMotor.set(ControlMode.PercentOutput, 1.1 * DriveConstants.kDriveKS);
  }

  public double getAngularVelocity() {
    return Conversions.falconToRPM(m_angleMotor.getSelectedSensorVelocity(), DriveConstants.kAngleGearRatio) * 2 * Math.PI / 60;
  }

  private void configDriveMotor() {
    m_driveMotor.configFactoryDefault();
    m_driveMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(
      DriveConstants.kDriveEnableCurrentLimit,
      DriveConstants.kDriveContinuousCurrentLimit,
      DriveConstants.kDrivePeakCurrentLimit,
      DriveConstants.kDrivePeakCurrentDuration
    ));
    m_driveMotor.config_kP(0, DriveConstants.kDriveP);
    m_driveMotor.config_kI(0, DriveConstants.kDriveI);
    m_driveMotor.config_kD(0, DriveConstants.kDriveD);
    m_driveMotor.config_kF(0, DriveConstants.kDriveF);
    m_driveMotor.configOpenloopRamp(DriveConstants.kOpenLoopRamp);
    m_driveMotor.configClosedloopRamp(DriveConstants.kClosedLoopRamp);
    m_driveMotor.setInverted(DriveConstants.kDriveMotorInvert);
    m_driveMotor.setNeutralMode(DriveConstants.kDriveNeutralMode);
    m_driveMotor.configVoltageCompSaturation(DriveConstants.kNormalOperatingVoltage);
    m_driveMotor.enableVoltageCompensation(true);
    m_driveMotor.setSelectedSensorPosition(0);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(
        Conversions.falconToMPS(m_driveMotor.getSelectedSensorVelocity(), DriveConstants.kWheelCircumference,
            DriveConstants.kDriveGearRatio),
        getAngle());
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        Conversions.falconToMeters(m_driveMotor.getSelectedSensorPosition(), DriveConstants.kWheelCircumference,
            DriveConstants.kDriveGearRatio),
        getAngle());
  }

  private void setupShuffleboard() {
    if (DriveConstants.kUseTelemetry) {
      m_swerveTab.addDouble(m_moduleAbbr + " CANcoder Angle (deg)", getCANcoder()::getDegrees);
      m_swerveTab.addDouble(m_moduleAbbr + " FX Angle (deg)", getPosition().angle::getDegrees);
      m_swerveTab.addDouble(m_moduleAbbr + " Velocity (m/s)", () -> getState().speedMetersPerSecond);
      m_swerveTab.addDouble(m_moduleAbbr + " Desired Velocity (m/s)", () -> m_desiredState.speedMetersPerSecond);
      m_swerveTab.addDouble(m_moduleAbbr + " Desired Angle (deg)", () -> m_desiredState.angle.getDegrees());
      m_swerveTab.addBoolean(m_moduleAbbr + " Jitter prevention enabled", () -> m_angleJitterPreventionEnabled);
      m_swerveTab.addDouble(m_moduleAbbr + " Drive Current (A)", () -> m_driveMotor.getSupplyCurrent());
      m_swerveTab.addDouble(m_moduleAbbr + " Angle Current (A)", () -> m_angleMotor.getSupplyCurrent());
    }
  }
}