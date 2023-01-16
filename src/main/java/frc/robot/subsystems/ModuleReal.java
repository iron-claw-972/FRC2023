package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import lib.ctre_shims.TalonEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.util.MotorFactory;
import frc.robot.util.PracticeModeType;
import frc.robot.util.ShuffleboardManager;

public class ModuleReal extends Module {
  private final WPI_TalonFX m_driveMotor;
  private final WPI_TalonFX m_steerMotor;

  private final TalonEncoder m_driveEncoder;
  private final WPI_CANCoder m_encoder;

  private final PIDController m_drivePIDController = new PIDController(Constants.drive.kDriveP, Constants.drive.kDriveI,
      Constants.drive.kDriveD);

  private ProfiledPIDController m_turningPIDController = new ProfiledPIDController(
      Constants.drive.kSteerP,
      Constants.drive.kSteerI,
      Constants.drive.kSteerD,
      new TrapezoidProfile.Constraints(
          Constants.drive.kMaxAngularSpeed, Constants.drive.kMaxAngularAccel));

  private SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(Constants.drive.kDriveKS,
      Constants.drive.kDriveKV);
  private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(Constants.drive.kSteerKS,
      Constants.drive.kSteerKV);
      
  private double m_offset = 0.0;
  
  public double driveOutput = 0;
  
  public double turnFeedforward = 0.0;
  public double turnOutput = 0.0;

  public ModuleReal(
    int driveMotorPort,
    int steerMotorPort,
    int encoderPort,
    double encoderOffset
  ) {
    
    super();
    m_driveMotor = MotorFactory.createTalonFX(driveMotorPort, Constants.kRioCAN, 40, 80, 1, NeutralMode.Brake);
    m_steerMotor = MotorFactory.createTalonFX(steerMotorPort, Constants.kCanivoreCAN, 30, 60, 1, NeutralMode.Brake);

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

    m_encoder.configFeedbackCoefficient(2 * Math.PI / Constants.kCANcoderResolution, "rad", SensorTimeBase.PerSecond);

    m_offset = encoderOffset;

    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    m_driveEncoder.setDistancePerPulse(
        2 * Math.PI * Constants.drive.kWheelRadius / Constants.drive.kDriveGearRatio / Constants.kEncoderResolution);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous. Factor in the offset amount.
    m_turningPIDController.enableContinuousInput(-Math.PI + m_offset, Math.PI + m_offset);

    m_steerMotor.setInverted(true);

    m_turningPIDController.reset(getAngle()); // reset the PID, and the Trapezoid motion profile needs to know the starting state
  }


  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    if (Math.abs(desiredState.speedMetersPerSecond) < 0.001 && Robot.shuffleboard.getPracticeModeType() != PracticeModeType.TUNE_HEADING_PID) {
      stop();
      return;
    }

    if (Robot.shuffleboard.getPracticeModeType() != PracticeModeType.TUNE_HEADING_PID) {
      // Optimize the reference state to avoid spinning further than 90 degrees
      desiredState = SwerveModuleState.optimize(desiredState, new Rotation2d(getAngle()));
    }

    // Calculate the drive output from the drive PID controller.
    driveOutput = m_drivePIDController.calculate(m_driveEncoder.getRate(), desiredState.speedMetersPerSecond);

    final double driveFeedforward = m_driveFeedforward.calculate(desiredState.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    turnOutput = m_turningPIDController.calculate(getAngle(), desiredState.angle.getRadians());

    turnFeedforward = m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

    m_driveMotor.setVoltage(driveOutput + driveFeedforward);
    m_steerMotor.setVoltage(turnOutput + turnFeedforward); // * Constants.kMaxVoltage / RobotController.getBatteryVoltage()
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(m_driveMotor.getSelectedSensorVelocity(), new Rotation2d(getAngle()));
  }

  public ProfiledPIDController getSteerPID() {
    return m_turningPIDController;
  }

  public void resetSteerPID(double angle) {
    m_turningPIDController.reset(angle);
  }

  @Override
  public double getAngle() {
    return m_encoder.getAbsolutePosition() - m_offset;
  }

  @Override
  public double getDriveVelocity() {
    return m_driveEncoder.getRate();
  }

  public PIDController getDrivePID() {
    return m_drivePIDController;
  }

  @Override
  public double getTurnFeedForward() {
    return turnFeedforward;
  }

  @Override
  public double getTurnOutput() {
    return turnOutput;
  }

  public void stop() {
    m_driveMotor.set(0);
    m_steerMotor.set(0);
  }

  public void setDriveVoltage(double voltage) {
    m_driveMotor.setVoltage(voltage);
  }

  public void setTurnVoltage(double voltage) {
    m_steerMotor.setVoltage(voltage);
  }

  public void characterize(double volts) {
    m_driveMotor.setVoltage(volts);
    m_steerMotor.setVoltage(m_turnFeedforward.calculate(Units.degreesToRadians(getAngle()), 0.0));
  }

  public double getCharacterizationVelocity() {
    return m_driveEncoder.getRate();
  }

  public void getShuffleboardFeedForwardValues(double staticFeedforward, double VelocityFeedforward){
    m_driveFeedforward = new SimpleMotorFeedforward(staticFeedforward, VelocityFeedforward);
  }

}