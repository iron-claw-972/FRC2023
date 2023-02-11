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
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.constants.swerve.DriveConstants;
import frc.robot.constants.swerve.ModuleConstants;
import frc.robot.constants.swerve.ModuleType;
import frc.robot.util.LogManager;
import frc.robot.util.MotorFactory;
import lib.ctre_shims.TalonEncoder;

public class Module {

  /**
   * Creates a swerve module, or a simulated one if running on the simulator.
   * 
   * @param moduleConstants the constants for the module
   * @param moduleTab the shuffleboard tab for the module
   */
  public static Module create(ModuleConstants moduleConstants, ShuffleboardTab moduleTab) {
    if (Robot.isReal()) {
      return new Module(moduleConstants, moduleTab);
    } else {
      return new ModuleSim(moduleConstants, moduleTab);
    }
  }

  private final WPI_TalonFX m_driveMotor;
  private final WPI_TalonFX m_steerMotor;

  private final TalonEncoder m_driveEncoder;
  private final WPI_CANCoder m_encoder;

  private PIDController m_drivePIDController;

  private ProfiledPIDController m_steerPIDController;

  private SimpleMotorFeedforward m_driveFeedforward;
  private SimpleMotorFeedforward m_steerFeedForward;
      
  private double m_offset = 0.0;
  
  private double m_drivePIDOutput = 0;
  private double m_driveFeedforwardOutput = 0;
  private double m_steerFeedForwardOutput = 0.0;
  private double m_steerPIDOutput = 0.0;

  private SwerveModuleState m_desieredState = new SwerveModuleState();
  ShuffleboardTab m_moduleTab;

  private MedianFilter m_driveVelocityMedianFilter = new MedianFilter(80);

  private boolean m_optimizeStates = false;

  private ModuleType m_moduleType;
  
  public Module(ModuleConstants moduleConstants, ShuffleboardTab moduleTab) {
    this(
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

  public Module(
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
    
    if (Robot.isReal()) {
      // TODO: The CANBus needs to be a constant because on the new 2023 bot, drive motors use Canivore, not rio
      m_driveMotor = MotorFactory.createTalonFX(driveMotorPort, DriveConstants.kConstants.kDriveMotorCAN);
      m_steerMotor = MotorFactory.createTalonFX(steerMotorPort, DriveConstants.kConstants.kSteerMotorCAN);
    } else {
      m_driveMotor = new WPI_TalonFX(driveMotorPort);
      m_steerMotor = new WPI_TalonFX(steerMotorPort);
    }

    m_moduleTab = moduleTab;
    m_moduleType = moduleType;
    
    m_driveMotor.setNeutralMode(NeutralMode.Brake);
    m_steerMotor.setNeutralMode(NeutralMode.Brake);

    m_driveEncoder = new TalonEncoder(m_driveMotor);
    m_encoder = new WPI_CANCoder(encoderPort, DriveConstants.kConstants.kEncoderCAN);

    m_drivePIDController = new PIDController(driveP, driveI,driveD);
    m_steerPIDController = new ProfiledPIDController(
      steerP,
      steerI,
      steerD,
      new TrapezoidProfile.Constraints(
        DriveConstants.kConstants.kMaxAngularSpeed, DriveConstants.kConstants.kMaxAngularAccel));

    // reset encoder to factory defaults, reset position to the measurement of the
    // absolute encoder
    // by default the CANcoder sets it's feedback coefficient to 0.087890625, to
    // make degrees.
    m_encoder.configFactoryDefault();
    m_encoder.setPositionToAbsolute();

    m_encoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);

    m_encoder.configFeedbackCoefficient(2 * Math.PI / Constants.kCancoderResolution, "rad", SensorTimeBase.PerSecond);

    m_offset = encoderOffset;

    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    m_driveEncoder.setDistancePerPulse(
        2 * Math.PI * DriveConstants.kConstants.kWheelRadius / DriveConstants.kConstants.kDriveGearRatio / Constants.kCancoderResolution);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous. Factor in the offset amount.
    m_steerPIDController.enableContinuousInput(-Math.PI, Math.PI);

    m_steerMotor.setInverted(true);

    m_steerPIDController.reset(getSteerAngle()); // reset the PID, and the Trapezoid motion profile needs to know the starting state

    m_driveFeedforward = new SimpleMotorFeedforward(driveFeedForwardKS, driveFeedForwardKV);
    m_steerFeedForward = new SimpleMotorFeedforward(steerFeedForwardKS, steerFeedForwardKV);
  
    LogManager.addDouble(m_steerMotor.getDescription() + " Steer Absolute Position", () -> m_encoder.getAbsolutePosition());
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
    if (Math.abs(desiredState.speedMetersPerSecond) < 0.001) {
      stop();
      return;
    }

    if (m_optimizeStates == true) {
      // Optimize the reference state to avoid spinning further than 90 degrees
      desiredState = SwerveModuleState.optimize(desiredState, new Rotation2d(getSteerAngle()));
    }

    //no need to set the desisier stae directly as the angle and velocity set desierd values
    setDriveVelocity(desiredState.speedMetersPerSecond);
    setSteerAngle(desiredState.angle);
  }

  /**
   * Sets the drive velocity of the module using PIDF once. Should be called repeatedly to be effective.
   * @param speedMetersPerSecond the drive velocity in m/s
   */
  public void setDriveVelocity(double speedMetersPerSecond) {
    m_desieredState.speedMetersPerSecond = speedMetersPerSecond;
    m_drivePIDOutput = m_drivePIDController.calculate(m_driveEncoder.getRate(), speedMetersPerSecond);
    m_driveFeedforwardOutput = m_driveFeedforward.calculate(speedMetersPerSecond);
    setDriveVoltage(m_drivePIDOutput + m_driveFeedforwardOutput);
  }

  /**
   * Sets the steer angle of the module using a profiled PIDF. Should be called repeatedly to be effective.
   * @param angle a Rotation2d object representing the angle the steer of the module should go to.
   */
  public void setSteerAngle(Rotation2d angle) {
    m_desieredState.angle = angle;
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
   * Gets the drive position of the module in meters, scaled from the internal Falcon encoder.
   * 
   * @return module drive position in meters.
   */
  public double getDrivePosition() {
      return m_driveEncoder.getDistance() * DriveConstants.kConstants.kDriveGearRatio * 2 * Math.PI * DriveConstants.kConstants.kWheelRadius;
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
    return MathUtil.angleModulus(m_encoder.getAbsolutePosition() - m_offset);
  }

  /**
   * Gets the difference between the last set goal of the steer profiled pid and the current angle.
   * @return the error in radians, from -pi to pi
   */
  public double getSteerAngleError() {
    return MathUtil.angleModulus(getSteerAngle() - m_desieredState.speedMetersPerSecond);
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

  public double getDriveVelocityError() {
    return m_desieredState.speedMetersPerSecond - getDriveVelocity();
  }

  public double selfFeedforwardCharacterazation() {
    return m_driveEncoder.getRate();
  }

  public double getDriveVelocityFiltered() {
    return m_driveVelocityMedianFilter.calculate(getDriveVelocity());
  }

  public double getSteerVelocity() {
    return m_encoder.getVelocity();
  }

  /**
   * Update the driveFeedforward values from Shuffleboard.
   * @param staticFeedforward static feedforward value from Shuffleboard
   * @param velocityFeedForward velocity feedforward value from Shuffleboard
   */
  public void setDriveFeedForwardValues(double staticFeedforward, double velocityFeedForward) {
    m_driveFeedforward = new SimpleMotorFeedforward(staticFeedforward, velocityFeedForward);
  }

  public void setSteerFeedForwardValues(double staticFeedforward, double velocityFeedForward) {
    m_steerFeedForward= new SimpleMotorFeedforward(staticFeedforward, velocityFeedForward);
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
    return m_encoder;
  }

  public double getSteerFeedForwardOutput() {
    return m_steerFeedForwardOutput;
  }

  public double getSteerOutput() {
    return m_steerPIDOutput;
  }
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

  public void periodic() {
    // This method will be called once per scheduler run, mainly only used for simulation
  }

  public ModuleType getModuleType(){
    return m_moduleType;
  }

  public double getDesieredVelocity(){
    return m_desieredState.speedMetersPerSecond;
  }

  public Rotation2d getDesieredAngle(){
    return m_desieredState.angle;
  }
  
  public void setupModulesShuffleboard(){
    m_moduleTab.addNumber("FL desired speed", () -> getDesieredVelocity());
    // Drive PID output
    m_moduleTab.addNumber("FL PID Output", () -> getDrivePIDOutput());
    // get drive velocity
    m_moduleTab.addNumber("Vel FL Raw", () -> getDriveVelocity());
    // drivePIDS
    m_moduleTab.add("Drive PID " + m_moduleType.getAbbreviation(), getDrivePID());
    // Median Filltered Velocity Values
    m_moduleTab.addNumber("Vel " + m_moduleType.getAbbreviation() + " Filtered", () -> getDriveVelocityFiltered());
    // Desired Steer angles
    m_moduleTab.addNumber( m_moduleType.getAbbreviation() + "desired angle", () -> getDesieredAngle().getRadians());
    // Steer angles
    m_moduleTab.addNumber("Angle " + m_moduleType.getAbbreviation(), () -> getSteerAngle());
    // Steer Velocity
    m_moduleTab.addNumber("Steer Vel " + m_moduleType.getAbbreviation(), () -> getSteerVelocity());
    //Steer PID
    m_moduleTab.add("Steer PID " + m_moduleType.getAbbreviation(), getSteerPID());
  }

}