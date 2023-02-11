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
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.ModuleConstants;
import frc.robot.util.LogManager;
import frc.robot.util.MotorFactory;
import lib.ctre_shims.TalonEncoder;

public class Module {

  /**
   * Creates a swerve module, or a simulated one if running on the simulator.
   * 
   * @param driveMotorID the ID of the drive motor
   * @param steerMotorID the ID of the steer motor
   * @param encoderID the id of the CANcoder for measuring the module's angle
   * @param steerOffset the offset of the CANcoder's angle
   * @param feedforwardKS the static feedforward of the drive motor
   * @param feedforwardKV the velocity feedforward of the drive motor
   * @return
   */
  public static Module create(ModuleConstants moduleConstants) {
    if (Robot.isReal()) {
      return new Module(moduleConstants);
    } else {
      return new ModuleSim(moduleConstants);
    }
  }
  public static Module create(
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
      double steerD
    ) {
    if (Robot.isReal()) { 
      return new Module(
          driveMotorPort,
          steerMotorPort,
          encoderPort,
          encoderOffset,
          driveFeedForwardKS,
          driveFeedForwardKV,
          driveP,
          driveI,
          driveD,
          steerFeedForwardKS,
          steerFeedForwardKV,
          steerP,
          steerI,
          steerD
        );
    } else {
      return new ModuleSim(driveMotorPort, steerMotorPort, encoderPort, encoderOffset, driveFeedForwardKS, driveFeedForwardKV);
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

  private double m_steerError = 0;
  
  public double m_drivePIDOutput = 0;
  public double m_driveFeedforwardOutput = 0;
  public double m_steerFeedForwardOutput = 0.0;
  public double m_steerPIDOutput = 0.0;

  public double m_desiredSpeed = 0;
  public double m_desiredAngle = 0;

  public MedianFilter m_driveVelocityMedianFilter = new MedianFilter(80);
  public Boolean setOptimize=false;
  public Module(ModuleConstants moduleConstants){
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
      moduleConstants.getSteerD()
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
    double steerD
  ) {
    
    if (Robot.isReal()) {
      // TODO: The CANBus needs to be a constant because on the new 2023 bot, drive motors use Canivore, not rio
      m_driveMotor = MotorFactory.createTalonFX(driveMotorPort, Constants.kRioCAN);
      m_steerMotor = MotorFactory.createTalonFX(steerMotorPort, Constants.kCanivoreCAN);
    } else {
      m_driveMotor = new WPI_TalonFX(driveMotorPort);
      m_steerMotor = new WPI_TalonFX(steerMotorPort);
    }
    
    m_driveMotor.setNeutralMode(NeutralMode.Brake);
    m_steerMotor.setNeutralMode(NeutralMode.Brake);

    m_driveEncoder = new TalonEncoder(m_driveMotor);
    m_encoder = new WPI_CANCoder(encoderPort, Constants.kCanivoreCAN);

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
    m_encoder.configFactoryDefault();
    m_encoder.setPositionToAbsolute();

    m_encoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);

    m_encoder.configFeedbackCoefficient(2 * Math.PI / Constants.kCancoderResolution, "rad", SensorTimeBase.PerSecond);

    m_offset = encoderOffset;

    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    m_driveEncoder.setDistancePerPulse(
        2 * Math.PI * DriveConstants.kWheelRadius / DriveConstants.kDriveGearRatio / Constants.kCancoderResolution);

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
    if (Math.abs(desiredState.speedMetersPerSecond) < 0.001) { //TODO are there cases where we don't want to do this for testing?
      stop();
      return;
    }

    if (setOptimize==true) {
      // Optimize the reference state to avoid spinning further than 90 degrees
      desiredState = SwerveModuleState.optimize(desiredState, new Rotation2d(getSteerAngle()));
    }
    setDriveVelocity(desiredState.speedMetersPerSecond);
    setSteerAngle(desiredState.angle);
  }

  public void setDriveVelocity(double speedMetersPerSecond){
    m_drivePIDOutput = m_drivePIDController.calculate(m_driveEncoder.getRate(), speedMetersPerSecond);
    m_driveFeedforwardOutput = m_driveFeedforward.calculate(speedMetersPerSecond);
    m_driveMotor.setVoltage(m_drivePIDOutput + m_driveFeedforwardOutput);
  }

  public void setSteerAngle(Rotation2d angle){
    // Calculate the steer motor output from the steer PID controller.
    m_steerPIDOutput = m_steerPIDController.calculate(getSteerAngle(), angle.getRadians());
    m_steerFeedForwardOutput = m_steerFeedForward.calculate(m_steerPIDController.getSetpoint().velocity);
    m_steerMotor.setVoltage(m_steerPIDOutput + m_steerFeedForwardOutput);// * Constants.kMaxVoltage / RobotController.getBatteryVoltage()
  }

  public void setDriveVoltage(double voltage){
    m_driveMotor.setVoltage(voltage);
  }
  public void setSteerVoltage(double voltage){
    m_steerMotor.setVoltage(voltage);
  }
  
  /**
   * Gets the drive position of the module.
   * 
   * @return module drive position
   */
  public double getDrivePosition() {
      return m_driveEncoder.getDistance() * DriveConstants.kDriveGearRatio * 2 * Math.PI * DriveConstants.kWheelRadius;
  }

  /**
   * Stops the module by setting both steer and drive motors to 0.
   */
  public void stop() {
    m_driveMotor.set(0);
    m_steerMotor.set(0);
  }

  /**
   * Returns the current state of the module.
   * @return the current state of the module
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
   * Gets the angle of the module.
   * @return encoder's absolute position - offset
   */
  public double getSteerAngle() {
    return MathUtil.angleModulus(m_encoder.getAbsolutePosition() - m_offset);
  }
  public double getSteerAngleError(){
    double posError = MathUtil.angleModulus(getSteerAngle() - m_steerPIDController.getGoal().position);
    double negError = MathUtil.angleModulus(m_steerPIDController.getGoal().position - getSteerAngle());
    if (Math.abs(posError) < Math.abs(negError)) return posError;
    return negError;
  }

  /**
   * Gets the drive velocity of the module.
   * @return the rate of the drive encoder
   */
  public double getDriveVelocity() {
    return m_driveEncoder.getRate();
  }
  public double selfFeedforwardCharacterazation() {
    
    return m_driveEncoder.getRate();
  }

  public double getDriveVelocityFiltered(){
    return m_driveVelocityMedianFilter.calculate(getDriveVelocity());
  }

  public double getSteerVelocity(){
    return m_encoder.getVelocity();
  }

  /**
   * Update the driveFeedforward values from Shuffleboard.
   * @param staticFeedforward static feedforward value from Shuffleboard
   * @param velocityFeedForward velocity feedforward value from Shuffleboard
   */
  public void setDriveFeedForwardValues(double staticFeedforward, double velocityFeedForward){
    m_driveFeedforward = new SimpleMotorFeedforward(staticFeedforward, velocityFeedForward);
  }
  public void setSteerFeedForwardValues(double staticFeedforward, double velocityFeedForward){
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
  public void setOptimize(Boolean setOptimize){
    this.setOptimize=setOptimize;
  }

  public double getBusToDriveVoltage(){
    return m_driveMotor.getBusVoltage();
  }

  public double getDriveOutputVoltage(){
    return m_driveMotor.getMotorOutputVoltage();
  }

  public double getBusToSteerVoltage(){
    return m_steerMotor.getBusVoltage();
  }

  public double getSteerOutputVoltage(){
    return m_steerMotor.getMotorOutputVoltage();
  }

  public void periodic() {
    // This method will be called once per scheduler run, mainly only used for simulation
  }

}