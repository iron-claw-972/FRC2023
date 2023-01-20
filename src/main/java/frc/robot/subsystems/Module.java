package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.constants.ModuleConstants;
import frc.robot.util.MotorFactory;
import frc.robot.util.TestType;
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
  public static Module create(int driveMotorID, int steerMotorID, int encoderID, double steerOffset, double feedforwardKS, double feedforwardKV) {
    if (Robot.isReal()) { 
      return new Module(driveMotorID, steerMotorID, encoderID, steerOffset, feedforwardKS, feedforwardKV);
    } else {
      return new ModuleSim(driveMotorID, steerMotorID, encoderID, steerOffset, feedforwardKS, feedforwardKV);
    }
  }

  private final WPI_TalonFX m_driveMotor;
  private final WPI_TalonFX m_steerMotor;

  private final TalonEncoder m_driveEncoder;
  private final WPI_CANCoder m_encoder;

  private final PIDController m_drivePIDController = new PIDController(Constants.drive.kDriveP, Constants.drive.kDriveI,
    Constants.drive.kDriveD);

  private ProfiledPIDController m_steerPIDController = new ProfiledPIDController(
    Constants.drive.kSteerP,
    Constants.drive.kSteerI,
    Constants.drive.kSteerD,
    new TrapezoidProfile.Constraints(
      Constants.drive.kMaxAngularSpeed, Constants.drive.kMaxAngularAccel));

  private SimpleMotorFeedforward m_driveFeedforward;
  private final SimpleMotorFeedforward m_steerFeedForward = new SimpleMotorFeedforward(Constants.drive.kSteerKS, Constants.drive.kSteerKV);
      
  private double m_offset = 0.0;
  
  public double m_driveOutput = 0;
  
  public double m_steerFeedForwardOutput = 0.0;
  public double m_steerOutput = 0.0;
  public MedianFilter m_medianFilter = new MedianFilter(10);

  public Module(ModuleConstants moduleConstants){
    this(
      moduleConstants.getDrivePort(),
      moduleConstants.getSteerPort(),
      moduleConstants.getEncoderPort(),
      moduleConstants.getSteerOffset(),
      moduleConstants.getDriveKS(),
      moduleConstants.getDriveKV()
    );
  }

  public Module(
    int driveMotorPort,
    int steerMotorPort,
    int encoderPort,
    double encoderOffset,
    double feedforwardKS,
    double doublefeedforwardKV
  ) {
    
    if (Robot.isReal()) {
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
        2 * Math.PI * Constants.drive.kWheelRadius / Constants.drive.kDriveGearRatio / Constants.kCancoderResolution);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous. Factor in the offset amount.
    m_steerPIDController.enableContinuousInput(-Math.PI + m_offset, Math.PI + m_offset);

    m_steerMotor.setInverted(true);

    m_steerPIDController.reset(getAngle()); // reset the PID, and the Trapezoid motion profile needs to know the starting state

    m_driveFeedforward = new SimpleMotorFeedforward(feedforwardKS, doublefeedforwardKV);
  }


  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    if (Math.abs(desiredState.speedMetersPerSecond) < 0.001 && Robot.shuffleboard.getTestModeType() != TestType.TUNE_HEADING_PID) {
      stop();
      return;
    }

    if (Robot.shuffleboard.getTestModeType() != TestType.TUNE_HEADING_PID) {
      // Optimize the reference state to avoid spinning further than 90 degrees
      desiredState = SwerveModuleState.optimize(desiredState, new Rotation2d(getAngle()));
    }

    // Calculate the drive output from the drive PID controller.
    m_driveOutput = m_drivePIDController.calculate(m_driveEncoder.getRate(), desiredState.speedMetersPerSecond);

    final double driveFeedforward = m_driveFeedforward.calculate(desiredState.speedMetersPerSecond);

    // Calculate the steer motor output from the steer PID controller.
    m_steerOutput = m_steerPIDController.calculate(getAngle(), desiredState.angle.getRadians());

    m_steerFeedForwardOutput = m_steerFeedForward.calculate(m_steerPIDController.getSetpoint().velocity);

    m_driveMotor.setVoltage(m_driveOutput + driveFeedforward);
    m_steerMotor.setVoltage(m_steerOutput + m_steerFeedForwardOutput); // * Constants.kMaxVoltage / RobotController.getBatteryVoltage()
  }
  
  /**
   * Gets the drive position of the module.
   * 
   * @return module drive position
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
   * Returns the current state of the module.
   * @return the current state of the module
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(m_driveMotor.getSelectedSensorVelocity(), new Rotation2d(getAngle()));
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
  public double getAngle() {
    return m_encoder.getAbsolutePosition() - m_offset;
  }

  /**
   * Gets the drive velocity of the module.
   * @return the rate of the drive encoder
   */
  public double getDriveVelocity() {
    return m_driveEncoder.getRate();
  }

  public double getDriveVelocityFilltered(){
    return m_medianFilter.calculate(getDriveVelocity());
  }

  /**
   * Update the driveFeedforward values from Shuffleboard.
   * @param staticFeedforward static feedforward value from Shuffleboard
   * @param velocityFeedForward velocity feedforward value from Shuffleboard
   */
  public void getShuffleboardFeedForwardValues(double staticFeedforward, double velocityFeedForward){
    m_driveFeedforward = new SimpleMotorFeedforward(staticFeedforward, velocityFeedForward);
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

  public double getDriveOutput() {
    return m_driveOutput;
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
    return m_steerOutput;
  }

}