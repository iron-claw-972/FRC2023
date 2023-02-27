package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.constants.Constants;
import frc.robot.constants.FalconConstants;
import frc.robot.constants.swerve.DriveConstants;
import frc.robot.constants.swerve.ModuleConstants;
import frc.robot.constants.swerve.ModuleType;
import frc.robot.util.LogManager;
import frc.robot.util.MotorFactory;
import lib.ctre_shims.TalonEncoder;

/**
 * Represents a swerve module for a swerve drivetrain.
 */
public class Module {

  /**
   * @param moduleConstants the constants for the module, found in {@link ModuleConstants}
   * @param moduleTab the shuffleboard tab for the module
   * @see ModuleConstants
   */
  public static Module create(ModuleConstants moduleConstants, ShuffleboardTab moduleTab) {
    return new Module(
      moduleConstants.getDrivePort(),
      moduleConstants.getSteerPort(),
      moduleConstants.getEncoderPort(),
      moduleConstants.getSteerOffset(),
      moduleConstants.getDriveKS(),
      moduleConstants.getDriveKV(),
      moduleConstants.getDriveP(),
      moduleConstants.getDriveI(),
      moduleConstants.getDriveD(),
      moduleConstants.getSteerKS(),
      moduleConstants.getSteerKV(),
      moduleConstants.getSteerP(),
      moduleConstants.getSteerI(),
      moduleConstants.getSteerD(),
      moduleConstants.getType(),
      moduleTab
    );
  }

  private final WPI_TalonFX m_driveMotor;
  private final WPI_TalonFX m_steerMotor;

  private final TalonEncoder m_driveEncoder;
  private final WPI_CANCoder m_steerEncoder;

  private PIDController m_drivePIDController;

  private ProfiledPIDController m_steerPIDController;

  private SimpleMotorFeedforward m_driveFeedforward;
  private SimpleMotorFeedforward m_steerFeedForward;
  private double m_driveFeedForwardKS;
  private double m_driveFeedForwardKV;
  private double m_steerFeedForwardKS;
  private double m_steerFeedForwardKV;
      
  private double m_offset = 0.0;
  
  private double m_drivePIDOutput = 0;
  private double m_driveFeedforwardOutput = 0;
  private double m_steerFeedForwardOutput = 0.0;
  private double m_steerPIDOutput = 0.0;

  private SwerveModuleState m_desiredState = new SwerveModuleState();
  private ShuffleboardTab m_moduleTab;

  private MedianFilter m_driveVelocityMedianFilter = new MedianFilter(80);

  private boolean m_optimizeStates = false;
  private boolean m_stateDeadband = true;

  private ModuleType m_moduleType;

  private Module(
    int driveMotorPort,
    int steerMotorPort,
    int encoderPort,
    double encoderOffset,
    double driveFeedForwardKS,
    double driveFeedForwardKV,
    double driveP,
    double driveI,
    double driveD,
    double steerFeedForwardKS,
    double steerFeedForwardKV,
    double steerP,
    double steerI,
    double steerD,
    ModuleType moduleType,
    ShuffleboardTab moduleTab
  ) {
    m_driveMotor = MotorFactory.createTalonFX(driveMotorPort, DriveConstants.kDriveMotorCAN);
    m_steerMotor = MotorFactory.createTalonFX(steerMotorPort, DriveConstants.kSteerMotorCAN);

    m_moduleTab = moduleTab;
    m_moduleType = moduleType;
    
    m_driveMotor.setNeutralMode(NeutralMode.Brake);
    m_steerMotor.setNeutralMode(NeutralMode.Brake);

    m_driveEncoder = new TalonEncoder(m_driveMotor);
    m_steerEncoder = new WPI_CANCoder(encoderPort, DriveConstants.kSteerEncoderCAN);

    m_drivePIDController = new PIDController(driveP, driveI,driveD);
    m_steerPIDController = new ProfiledPIDController(
      steerP,
      steerI,
      steerD,
      new TrapezoidProfile.Constraints(
        DriveConstants.kMaxAngularSpeed, DriveConstants.kMaxAngularAccel));

    // reset encoder to factory defaults, reset position to the measurement of the
    // absolute encoder
    // by default the CANcoder sets it's feedback coefficient to 0.087890625, to
    // make degrees.
    m_steerEncoder.configFactoryDefault();
    m_steerEncoder.setPositionToAbsolute();

    // CANcoder from -180 to 180, then convert to rad -> output range is -pi to pi
    m_steerEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    m_steerEncoder.configFeedbackCoefficient(2 * Math.PI / Constants.kCancoderResolution, "rad", SensorTimeBase.PerSecond);

    m_offset = encoderOffset;

    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    m_driveEncoder.setDistancePerPulse(
        2 * Math.PI * DriveConstants.kWheelRadius / DriveConstants.kDriveGearRatio / FalconConstants.kResolution);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous. Factor in the offset amount.
    m_steerPIDController.enableContinuousInput(-Math.PI, Math.PI);

    m_steerMotor.setInverted(true);

    m_steerPIDController.reset(getSteerAngle()); // reset the PID, and the Trapezoid motion profile needs to know the starting state

    m_driveFeedForwardKS = driveFeedForwardKS;
    m_driveFeedForwardKV = driveFeedForwardKV;
    m_steerFeedForwardKS = steerFeedForwardKS;
    m_steerFeedForwardKV = steerFeedForwardKV;

    m_driveFeedforward = new SimpleMotorFeedforward(m_driveFeedForwardKS, m_driveFeedForwardKV);
    m_steerFeedForward = new SimpleMotorFeedforward(m_steerFeedForwardKS, m_steerFeedForwardKV);
  
    LogManager.addDouble(m_steerMotor.getDescription() + " Steer Absolute Position", () -> m_steerEncoder.getAbsolutePosition());
    LogManager.addDouble(m_steerMotor.getDescription() + " Steer Velocity", () -> getSteerVelocity());
    LogManager.addDouble(m_steerMotor.getDescription() + " Steer Error", () -> getSteerAngleError());
    LogManager.addDouble(m_steerMotor.getDescription() + " Steer Voltage", () -> getSteerOutputVoltage());
    LogManager.addDouble(m_steerMotor.getDescription() + " Bus to Steer Voltage", () -> getBusToSteerVoltage());
    LogManager.addDouble(m_driveMotor.getDescription() + " Drive Velocity Filtered", () -> getDriveVelocityFiltered());
    LogManager.addDouble(m_driveMotor.getDescription() + " Drive Voltage", () -> getDriveOutputVoltage());
    LogManager.addDouble(m_driveMotor.getDescription() + " Bus to Drive Voltage", () -> getBusToDriveVoltage());
  }


  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    if (Math.abs(desiredState.speedMetersPerSecond) < 0.001 && m_stateDeadband) {
      stop();
      return;
    }

    if (m_optimizeStates) {
      // Optimize the reference state to avoid spinning further than 90 degrees
      desiredState = SwerveModuleState.optimize(desiredState, new Rotation2d(getSteerAngle()));
    }

    //no need to set the desired state directly as the angle and velocity set desired values
    setDriveVelocity(desiredState.speedMetersPerSecond);
    setSteerAngle(desiredState.angle);
  }

  /**
   * Sets the drive velocity of the module using PIDF once. Should be called repeatedly to be effective.
   * @param speedMetersPerSecond the drive velocity in m/s.
   */
  public void setDriveVelocity(double speedMetersPerSecond) {
    m_desiredState.speedMetersPerSecond = speedMetersPerSecond;
    m_drivePIDOutput = m_drivePIDController.calculate(m_driveEncoder.getRate(), speedMetersPerSecond);
    m_driveFeedforwardOutput = m_driveFeedforward.calculate(speedMetersPerSecond);
    setDriveVoltage(m_drivePIDOutput + m_driveFeedforwardOutput);
  }

  /**
   * Sets the steer angle of the module using a profiled PIDF. Should be called repeatedly to be effective.
   * @param angle a Rotation2d object representing the angle the steer of the module should go to.
   */
  public void setSteerAngle(Rotation2d angle) {
    m_desiredState.angle = angle;
    // Calculate the steer motor output from the steer PID controller.
    m_steerPIDOutput = m_steerPIDController.calculate(getSteerAngle(), MathUtil.angleModulus(angle.getRadians()));
    m_steerFeedForwardOutput = m_steerFeedForward.calculate(m_steerPIDController.getSetpoint().velocity);
    setSteerVoltage(m_steerPIDOutput + m_steerFeedForwardOutput);// * Constants.kMaxVoltage / RobotController.getBatteryVoltage()
  }

  /**
   * Directly sets the voltage of the drive motor, should be called repeatedly.
   * @param voltage the voltage of the drive motor.
   */
  public void setDriveVoltage(double voltage) {
    m_driveMotor.setVoltage(voltage);
  }

  /**
   * Directly sets the voltage of the steer motor, should be called repeatedly.
   * @param voltage the voltage of the steer motor.
   */
  public void setSteerVoltage(double voltage) {
    m_steerMotor.setVoltage(voltage);
  }

  /**
   * Enables or disables the state deadband for this swerve module. 
   * The state deadband determines if this module will stop drive and steer motors when inputted drive velocity is low. 
   * It should be enabled for all regular driving, to prevent releasing the controls from setting the angles.
   */
  public void enableStateDeadband(boolean stateDeadband){
    m_stateDeadband = stateDeadband;
  }
  
  /**
   * Gets the drive position of the module in meters, scaled from the internal Falcon encoder.
   * 
   * @return module drive position in meters.
   */
  public double getDrivePosition() {
      return m_driveEncoder.getDistance();
  }

  /**
   * Stops the module by setting both steer and drive motors to 0.
   */
  public void stop() {
    m_driveMotor.set(0);
    m_steerMotor.set(0);
  }

  /**
   * Returns the current drive velocity and steer angle of the module. 
   * Velocity comes from the internal drive motor encoder, angle from the CANcoder.
   * @return A SwerveModuleState object with the current velocity and angle.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(m_driveMotor.getSelectedSensorVelocity(), new Rotation2d(getSteerAngle()));
  }

  /**
   * Resets the steer PID controller.
   * @param angle current position of the PID controller
   */
  public void resetSteerPID(double angle) {
    m_steerPIDController.reset(angle);
  }
  
  /**
   * Gets the angle of the module from the CANcoder with the offset specified by the constructor to ensure 0 is forward for all modules.
   * @return encoder's position in radians, from -pi to pi
   */
  public double getSteerAngle() {
    return MathUtil.angleModulus(m_steerEncoder.getAbsolutePosition() - m_offset);
  }

  /**
   * Gets the difference between the last set goal of the steer profiled pid and the current angle.
   * @return the error in radians, from -pi to pi
   */
  public double getSteerAngleError() {
    return MathUtil.angleModulus(getSteerAngle() - m_desiredState.speedMetersPerSecond);
  }

  /**
   * Returns the position and angle of the module. The position is in meters, and it is from the internal encoder of the drive motor.
   * @return a SwerveModulePosition object with the distance traveled by the drive motor and the current steer angle.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getSteerAngle()));
  }

  /**
   * Gets the drive velocity of the module.
   * @return the rate of the drive encoder
   */
  public double getDriveVelocity() {
    return m_driveEncoder.getRate();
  }

  /**
   * Gets the difference between the last set goal of the drive velocity and the current velocity.
   * @return the error in m/s
   */
  public double getDriveVelocityError() {
    return m_desiredState.speedMetersPerSecond - getDriveVelocity();
  }

  /**
   * Gets the drive velocity of the module, filtered by a median filter.
   * @return the rate of the drive encoder, filtered by a median filter
   */
  public double getDriveVelocityFiltered() {
    return m_driveVelocityMedianFilter.calculate(getDriveVelocity());
  }

  /**
   * Gets the steer velocity of the module.
   * @return the velocity of the steer encoder
   */
  public double getSteerVelocity() {
    return m_steerEncoder.getVelocity();
  }

  /**
   * Update the driveFeedforward values from Shuffleboard.
   * @param staticFeedforward static feedforward value from Shuffleboard
   * @param velocityFeedForward velocity feedforward value from Shuffleboard
   */
  public void setDriveFeedForwardValues(double staticFeedforward, double velocityFeedForward) {
    m_steerFeedForwardKS = staticFeedforward;
    m_steerFeedForwardKV = velocityFeedForward;
    m_driveFeedforward = new SimpleMotorFeedforward(staticFeedforward, velocityFeedForward);
  }

  /**
   * Update the steerFeedforward values from Shuffleboard.
   * @param staticFeedforward static feedforward value from Shuffleboard
   * @param velocityFeedForward velocity feedforward value from Shuffleboard
   */
  public void setSteerFeedForwardValues(double staticFeedforward, double velocityFeedForward) {
    m_steerFeedForwardKS = staticFeedforward;
    m_steerFeedForwardKV = velocityFeedForward;
    m_steerFeedForward = new SimpleMotorFeedforward(staticFeedforward, velocityFeedForward);
  }
  
  // Getter Methods
  public ProfiledPIDController getSteerPID() {
    return m_steerPIDController;
  }

  public PIDController getDrivePID() {
    return m_drivePIDController;
  }

  public SimpleMotorFeedforward getDriveFeedforward() {
    return m_driveFeedforward;
  }

  public SimpleMotorFeedforward getSteerFeedforward() {
    return m_steerFeedForward;
  }

  public double getDrivePIDOutput() {
    return m_drivePIDOutput;
  }

  public WPI_TalonFX getDriveMotor() {
    return m_driveMotor;
  }

  public WPI_TalonFX getSteerMotor() {
    return m_steerMotor;
  }

  public TalonEncoder getDriveEncoder() {
    return m_driveEncoder;
  }

  public WPI_CANCoder getEncoder() {
    return m_steerEncoder;
  }

  public double getSteerFeedForwardOutput() {
    return m_steerFeedForwardOutput;
  }

  public double getSteerOutput() {
    return m_steerPIDOutput;
  }

  /**
   * Sets the optimize state for this swerve module.
   * Optimizing the state means the module will not turn the steer motors more than 90 degrees for any one movement.
   */
  public void setOptimize(Boolean setOptimize) {
    this.m_optimizeStates = setOptimize;
  }

  public double getBusToDriveVoltage() {
    return m_driveMotor.getBusVoltage();
  }

  public double getDriveOutputVoltage() {
    return m_driveMotor.getMotorOutputVoltage();
  }

  public double getBusToSteerVoltage() {
    return m_steerMotor.getBusVoltage();
  }

  public double getSteerOutputVoltage() {
    return m_steerMotor.getMotorOutputVoltage();
  }

  public ModuleType getModuleType() {
    return m_moduleType;
  }
  public int getId(){
    return getModuleType().id;
  }

  public double getDesiredVelocity() {
    return m_desiredState.speedMetersPerSecond;
  }

  public Rotation2d getDesiredAngle() {
    return m_desiredState.angle;
  }

  public double getDriveFeedForwardKS() {
    return m_driveFeedForwardKS;
  }

  public double getDriveFeedForwardKV() {
    return m_driveFeedForwardKV;
  }

  public double getSteerFeedForwardKS() {
    return m_steerFeedForwardKS;
  }
  
  public double getSteerFeedForwardKV() {
    return m_steerFeedForwardKV;
  }
  
  /**
   * Sets up the Shuffleboard tab for the module.
   */
  public void setupModulesShuffleboard() {
    m_moduleTab.addNumber(m_moduleType.abbrev + " desired speed", () -> getDesiredVelocity());
    // Drive PID output
    m_moduleTab.addNumber(m_moduleType.abbrev + " PID Output", () -> getDrivePIDOutput());
    // get drive velocity
    m_moduleTab.addNumber("Vel " + m_moduleType.abbrev + " Raw", () -> getDriveVelocity());
    // drivePIDs
    m_moduleTab.add("Drive PID " + m_moduleType.abbrev, getDrivePID());
    // Median Filtered Velocity Values
    m_moduleTab.addNumber("Vel " + m_moduleType.abbrev + " Filtered", () -> getDriveVelocityFiltered());
    // Desired Steer angles
    m_moduleTab.addNumber(m_moduleType.abbrev + " desired angle", () -> getDesiredAngle().getRadians());
    // Steer angles
    m_moduleTab.addNumber("Angle " + m_moduleType.abbrev, () -> getSteerAngle());
    // Steer Velocity
    m_moduleTab.addNumber("Steer Vel " + m_moduleType.abbrev, () -> getSteerVelocity());
    //Steer PID
    m_moduleTab.add("Steer PID " + m_moduleType.abbrev, getSteerPID());
  }

}