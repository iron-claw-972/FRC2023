package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoderSimCollection;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.constants.Constants;
import frc.robot.constants.ModuleConstants;
import lib.ctre_shims.TalonEncoderSim;

/**
 * Swerve module for drivetrain to be used inside of simulation.
 * 
 * @see frc.robot.subsystems.Module
 */
public class ModuleSim extends Module {

  private final FlywheelSim m_driveMotorSim = new FlywheelSim(DCMotor.getFalcon500(1), Constants.drive.kDriveGearRatio, 0.025);
  private final FlywheelSim m_steerMotorSim = new FlywheelSim(DCMotor.getFalcon500(1), Constants.drive.kSteerGearRatio, 0.004096955);

  private final TalonEncoderSim m_driveEncoderSim;
  private final CANCoderSimCollection m_encoderSim;

  private double m_currentSteerPositionRad = 0;

  public ModuleSim(
    ModuleConstants moduleConstants
  ) {
    this(
      moduleConstants.getDrivePort(),
      moduleConstants.getSteerPort(),
      moduleConstants.getEncoderPort(),
      moduleConstants.getSteerOffset(),
      moduleConstants.getDriveKS(),
      moduleConstants.getDriveKV()
    );
  }

  public ModuleSim(
    int driveMotorPort,
    int steerMotorPort,
    int encoderPort,
    double encoderOffset,
    double feedforwardKS,
    double feedforwardKV
  ) {

    super(driveMotorPort, steerMotorPort, encoderPort, encoderOffset, feedforwardKS, feedforwardKV,0,0,0,0,0,0,0,0);
    m_driveEncoderSim = new TalonEncoderSim(getDriveEncoder());
    m_encoderSim = new CANCoderSimCollection(getEncoder());

  }

  /**
   * Returns the current state of the module.
   * @return the current state of the module
   */
  @Override
  public SwerveModuleState getState() {
    return new SwerveModuleState(m_driveMotorSim.getAngularVelocityRPM() * Constants.drive.kWheelRadius * 2 * Math.PI / 60, new Rotation2d(m_currentSteerPositionRad));
  }

  /**
   * Updates the simulation.
   */
  @Override
  public void periodic() {
    m_driveMotorSim.update(0.02);
    m_steerMotorSim.update(0.02);

    double angleDiffRad = m_steerMotorSim.getAngularVelocityRPM() * 0.02;
    m_currentSteerPositionRad += angleDiffRad;

    // System.out.println(m_currentSteerPositionRad);
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  @Override
  public void setDesiredState(SwerveModuleState desiredState) {
    if (Math.abs(desiredState.speedMetersPerSecond) < 0.001) {
      stop();
      return;
    }
    // Optimize the reference state to avoid spinning further than 90 degrees
    desiredState = SwerveModuleState.optimize(desiredState, new Rotation2d(m_currentSteerPositionRad));

    // Calculate the drive output from the drive PID controller.
    final double driveOutput = getDrivePID().calculate(getDriveEncoder().getRate(),
            desiredState.speedMetersPerSecond);

    final double driveFeedforward = getDriveFeedforward().calculate(desiredState.speedMetersPerSecond);

    final double turnOutput = getSteerPID().calculate(m_currentSteerPositionRad,
            desiredState.angle.getRadians());

    final double turnFeedforward = getSteerFeedforward().calculate(getSteerPID().getSetpoint().velocity);

    m_driveMotorSim.setInputVoltage(driveOutput * 5 + driveFeedforward);
    m_steerMotorSim.setInputVoltage(turnOutput + turnFeedforward);
  }

  /**
   * Gets the simulated angle of the module.
   */
  @Override
  public double getSteerAngle() {
    return m_currentSteerPositionRad;
  }

}