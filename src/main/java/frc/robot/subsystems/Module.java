package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.swerve.DriveConstants;
import frc.robot.constants.swerve.ModuleConstants;
import frc.robot.util.Conversions;
import lib.CTREModuleState;

public class Module extends SubsystemBase {
  private final ShuffleboardTab m_swerveTab;

  private final int m_moduleIndex;
  private final Rotation2d m_angleOffset;

  private final String m_moduleAbbr;

  private final WPI_TalonFX m_angleMotor;
  private final WPI_TalonFX m_driveMotor;
  private final WPI_CANCoder m_CANcoder;
  private SwerveModuleState m_desiredState;

  private boolean m_stateDeadband = true;
  private boolean m_optimizeStates = true;

  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(DriveConstants.kDriveKS, DriveConstants.kDriveKV, DriveConstants.kDriveKA);


  public Module(ModuleConstants moduleConstants, ShuffleboardTab swerveTab) {
    m_swerveTab = swerveTab;
    m_moduleIndex = moduleConstants.getType().id;
    m_moduleAbbr = moduleConstants.getType().abbrev;
    m_angleOffset = new Rotation2d(moduleConstants.getSteerOffset());

    /* Angle Encoder Config */
    m_CANcoder = new WPI_CANCoder(moduleConstants.getEncoderPort(), DriveConstants.kSteerEncoderCAN);
    configCANcoder();

    /* Angle Motor Config */
    m_angleMotor = new WPI_TalonFX(moduleConstants.getSteerPort(), DriveConstants.kSteerEncoderCAN);
    configAngleMotor();

    /* Drive Motor Config */
    m_driveMotor = new WPI_TalonFX(moduleConstants.getDrivePort(), DriveConstants.kDriveMotorCAN);
    configDriveMotor();

    setDesiredState(new SwerveModuleState(0, getAngle()), false);

    setupShuffleboard();
  }

  
  @Override
  public void periodic() {
    
  }

  private void configCANcoder() {
    m_CANcoder.configFactoryDefault();
    m_CANcoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
    m_CANcoder.configSensorDirection(DriveConstants.kModuleConstants.canCoderInvert);
    m_CANcoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
    m_CANcoder.configFeedbackCoefficient(360 / 4096, "deg", SensorTimeBase.PerSecond);
  }

  /**
   * resets steer motor's integrated encoder using CANcoder's reading
   */
  public void resetToAbsolute() {
    m_angleMotor.setSelectedSensorPosition(Conversions.degreesToFalcon(
      getCANcoder().getDegrees(),
      DriveConstants.kAngleGearRatio
    ));
    
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
    m_driveMotor.configVoltageCompSaturation(Constants.kRobotVoltage);
    m_driveMotor.enableVoltageCompensation(true);
    m_driveMotor.setSelectedSensorPosition(0);
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
    m_angleMotor.configVoltageCompSaturation(Constants.kRobotVoltage);
    m_angleMotor.enableVoltageCompensation(true);
    resetToAbsolute();
  }

  /**
   * set module desired state
   * @param desiredState state to set module to
   * @param isOpenLoop don't use integrated velocity PID and instead set percentage power for state velocity
   */
  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    /*
     * This is a custom optimize function, since default WPILib optimize assumes
     * continuous controller which CTRE and Rev onboard is not
     */
    desiredState = m_optimizeStates ? CTREModuleState.optimize(desiredState, getState().angle) : desiredState;
    m_desiredState = desiredState;
    setAngle(desiredState);
    setSpeed(desiredState, isOpenLoop);
  }
    /**
   * sets desired state velocity
   * @param desiredState desired state with desired velocity
   * @param isOpenLoop don't use integrated velocity PID and instead set percentage power
   */
  private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
    // TODO: why is open loop even a option check if needed for driver
    if (isOpenLoop) {
      double percentOutput = desiredState.speedMetersPerSecond / DriveConstants.kMaxSpeed;
      m_driveMotor.set(ControlMode.PercentOutput, percentOutput);
    } else {
      double velocity = Conversions.MPSToFalcon(
        desiredState.speedMetersPerSecond, 
        DriveConstants.kWheelCircumference,
        DriveConstants.kDriveGearRatio);
      m_driveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward,
          feedforward.calculate(desiredState.speedMetersPerSecond));
    }
  }
  /**
   * sets desired angle base off desired state. Will not move motor is desired speed is too low.
   * @param desiredState desired state of swerve module
   */
  private void setAngle(SwerveModuleState desiredState) {
    // Prevent rotating module if desired speed < 1% of max speed. Prevents Jittering.
    if (m_stateDeadband && (Math.abs(desiredState.speedMetersPerSecond) <= (DriveConstants.kMaxSpeed * 0.01))) {
      stop();
      return;
    }

    setAngle(desiredState.angle);
  }
  /**
   * rotates swerve module steer to desired angle
   * @param rotation2d desired steer angle
   * @param rotationBlind should the angle be interpreted continuously
   */
  public void setAngle(Rotation2d rotation2d, boolean rotationBlind) {
    double angle = rotation2d.getRadians();
    if (rotationBlind) angle = MathUtil.inputModulus(
      angle, 
      getAngle().getRadians() - Math.PI,
      getAngle().getRadians() + Math.PI
    );

    m_desiredState.angle = new Rotation2d(angle);

    m_angleMotor.set(ControlMode.Position, Conversions.degreesToFalcon(
      rotation2d.getDegrees(), DriveConstants.kAngleGearRatio
    ));
  }
  /**
   * rotates swerve module steer to desired angle without the angle being interpreted continuously
   * @param rotation2d desired steer angle
   */
  public void setAngle(Rotation2d rotation2d) {
    setAngle(rotation2d, false);
  }
  /**
   * set module motor powers to 0
   */
  public void stop() {
    m_driveMotor.set(0);
    m_angleMotor.set(0);
  }
  /**
   * sets voltage of drive motor
   * @param voltage voltage to set drive motor to in volts
   */
  public void setDriveVoltage(double voltage) {
    m_driveMotor.set(ControlMode.PercentOutput, voltage / Constants.kRobotVoltage);
  }
    /**
   * sets voltage of steer motor
   * @param voltage voltage to set steer motor to in volts
   */
  public void setSteerVoltage(double voltage) {
    m_angleMotor.set(ControlMode.PercentOutput, voltage / Constants.kRobotVoltage);
  }

  /**
   * sets state deadband enable status used for not running the module if desired speed is too low
   * @param enable should state deadband be enabled
   */
  public void enableStateDeadband(boolean enabled) {
    m_stateDeadband = enabled;
  }
  /**
   * sets swerve module state optimization enable status used to find least steer angle rotation needed
   * @param enable should state optimization be enabled
   */
  public void setOptimize(boolean enable) {
    m_optimizeStates = enable;
  }
  
  /**
   * @return index in drivetrain module array
   */
  public int getModuleIndex() {
    return m_moduleIndex;
  }
  
  /**
   * @return current state of module
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        getDriveVelocity(),
        getAngle());
  }
  /**
   * @return desired/ target state of module
   */
  public SwerveModuleState getDesiredState() {
    return m_desiredState;
  }
  /**
   * @return current swerve module position 
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        getDrivePosition(),
        getAngle());
  }

  /**
   * @return desired drive velocity
   */
  public double getDesiredVelocity() {
    return getDesiredState().speedMetersPerSecond;
  }
  /**
   * @return drive velocity in meters per second
   */
  public double getDriveVelocity() {
    return Conversions.falconToMPS(
      m_driveMotor.getSelectedSensorVelocity(), 
      DriveConstants.kWheelCircumference,
      DriveConstants.kDriveGearRatio
    );
  }
  /**
   * @return difference between desired and actual drive velocity in meters per second
   */
  public double getDriveVelocityError() {
    return m_desiredState.speedMetersPerSecond - getDriveVelocity();
  }
  /**
   * @return get encoder position in meters
   */
  public double getDrivePosition(){
    return Conversions.falconToMeters(
          m_driveMotor.getSelectedSensorPosition(),
          DriveConstants.kWheelCircumference,
          DriveConstants.kDriveGearRatio);
  }
  
  /**
   * @return module steer angle from Cancoder as Rotation2d with 0 being straight forward
   */
  public Rotation2d getCANcoder() {
    return Rotation2d.fromDegrees(m_CANcoder.getAbsolutePosition() - m_angleOffset.getDegrees());
  }
  /**
   * @return module steer angle from steer motor's integrated encoder as Rotation2d with 0 being straight forward
   */
  public Rotation2d getAngle() {
    return Rotation2d.fromDegrees( Conversions.falconToDegrees(
      m_angleMotor.getSelectedSensorPosition(), 
      DriveConstants.kAngleGearRatio
    ));
  }
  /**
   * @return desired module steer angle
   */
  public Rotation2d getDesiredAngle() {
    return getDesiredState().angle;
  }
  /**
   * @return module steer rotational velocity in radians per second
   */
  public double getSteerVelocity() {
    return Conversions.falconToRPM(m_angleMotor.getSelectedSensorVelocity(), DriveConstants.kAngleGearRatio) * 2 * Math.PI / 60;
  }

  private void setupShuffleboard() {
    if (Constants.kUseTelemetry && RobotBase.isReal()) {
      m_swerveTab.addDouble(m_moduleAbbr + " CANcoder Angle (deg)", () -> getCANcoder().getDegrees());
      m_swerveTab.addDouble(m_moduleAbbr + " FX Angle (deg)", () -> getAngle().getDegrees());
      m_swerveTab.addDouble(m_moduleAbbr + " Velocity (m/s)", () -> getDriveVelocity());
      m_swerveTab.addDouble(m_moduleAbbr + " Desired Velocity (m/s)", () -> m_desiredState.speedMetersPerSecond);
      m_swerveTab.addDouble(m_moduleAbbr + " Desired Angle (deg)", () -> m_desiredState.angle.getDegrees());
      m_swerveTab.addBoolean(m_moduleAbbr + " Jitter prevention enabled", () -> m_stateDeadband);
      m_swerveTab.addDouble(m_moduleAbbr + " Drive Current (A)", () -> m_driveMotor.getSupplyCurrent());
      m_swerveTab.addDouble(m_moduleAbbr + " Angle Current (A)", () -> m_angleMotor.getSupplyCurrent());
    }
  }

  //TODO: Possibly reimplement custom feedforward characterization
  public void setDriveFeedForwardValues(double kS, double kV) {    
  }
  public double getSteerFeedForwardKV() {
    return 0;
  }
  public double getSteerFeedForwardKS() {
    return 0;
  }
  public double getDriveFeedForwardKV() {
    return DriveConstants.kDriveKV;
  }
  public double getDriveFeedForwardKS() {
    return DriveConstants.kDriveKS;
  }
  
}