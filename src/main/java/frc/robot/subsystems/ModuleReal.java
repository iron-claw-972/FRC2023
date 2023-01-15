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

public class ModuleReal extends Module {
      
  private double m_offset = 0.0;
  
  public double m_driveOutput = 0;
  
  public double m_steerFeedforward = 0.0;
  public double m_steerOutput = 0.0;

  public ModuleReal(
    int driveMotorPort,
    int steerMotorPort,
    int encoderPort,
    double encoderOffset
  ) {
    
    super(driveMotorPort, steerMotorPort, encoderPort, encoderOffset);
  
    // reset encoder to factory defaults, reset position to the measurement of the
    // absolute encoder
    // by default the CANcoder sets it's feedback coefficient to 0.087890625, to
    // make degrees.
    getEncoder().configFactoryDefault();
    getEncoder().setPositionToAbsolute();

    getEncoder().configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);

    getEncoder().configFeedbackCoefficient(2 * Math.PI / Constants.kCANcoderResolution, "rad", SensorTimeBase.PerSecond);

    m_offset = encoderOffset;

    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    getDriveEncoder().setDistancePerPulse(
        2 * Math.PI * Constants.drive.kWheelRadius / Constants.drive.kDriveGearRatio / Constants.kEncoderResolution);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous. Factor in the offset amount.
    getSteerPIDController().enableContinuousInput(-Math.PI + m_offset, Math.PI + m_offset);

    getSteerMotor().setInverted(true);

    getSteerPIDController().reset(getAngle()); // reset the PID, and the Trapezoid motion profile needs to know the starting state
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
    m_driveOutput = getDrivePIDController().calculate(getDriveEncoder().getRate(), desiredState.speedMetersPerSecond);

    final double driveFeedforward = getDriveFeedforward().calculate(desiredState.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    m_steerOutput = getSteerPIDController().calculate(getAngle(), desiredState.angle.getRadians());

    m_steerFeedforward = getSteerFeedforward().calculate(getSteerPIDController().getSetpoint().velocity);

    getDriveMotor().setVoltage(m_driveOutput + driveFeedforward);
    getSteerMotor().setVoltage(m_steerOutput + m_steerFeedforward); // * Constants.kMaxVoltage / RobotController.getBatteryVoltage()
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveMotor().getSelectedSensorVelocity(), new Rotation2d(getAngle()));
  }

  public void resetSteerPID(double angle) {
    getSteerPIDController().reset(angle);
  }

  @Override
  public double getAngle() {
    return getEncoder().getAbsolutePosition() - m_offset;
  }

  @Override
  public double getDriveVelocity() {
    return getDriveEncoder().getRate();
  }

  // TODO: FIGURE OUT THIS
  @Override
  public double getTurnFeedForward() {
    return m_steerFeedforward;
  }

  @Override
  public double getM_steerOutput() {
    return m_steerOutput;
  }

  public void stop() {
    getDriveMotor().set(0);
    getSteerMotor().set(0);
  }

  public void setDriveVoltage(double voltage) {
    getDriveMotor().setVoltage(voltage);
  }

  public void setTurnVoltage(double voltage) {
    getSteerMotor().setVoltage(voltage);
  }

  public void characterize(double volts) {
    getDriveMotor().setVoltage(volts);
    getSteerMotor().setVoltage(getSteerFeedforward().calculate(Units.degreesToRadians(getAngle()), 0.0));
  }

  public double getCharacterizationVelocity() {
    return getDriveEncoder().getRate();
  }

}