package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoderSimCollection;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.constants.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.Constants;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.ModuleConstants;
import lib.ctre_shims.TalonEncoderSim;

/**
 * Swerve module for drivetrain to be used inside of simulation.
 * 
 * @see frc.robot.subsystems.Module
 */
public class ModuleSim extends Module {

  // private final FlywheelSim m_driveMotorSim = new FlywheelSim(DCMotor.getFalcon500(1), DriveConstants.kDriveGearRatio, 0.025);
  // private final FlywheelSim m_steerMotorSim = new FlywheelSim(DCMotor.getFalcon500(1), DriveConstants.kSteerGearRatio, 0.004096955);

  private final TalonEncoderSim m_driveEncoderSim;
  private final CANCoderSimCollection m_encoderSim;

  private double m_currentSteerPositionRad = 0;
  private double m_currentDrivePositionMeters = 0;
  private double m_currentSpeed = 0;

  public ModuleSim(
    ModuleConstants moduleConstants, ShuffleboardTab moduleTab
  ) {
    this(
      moduleConstants.getDrivePort(),
      moduleConstants.getSteerPort(),
      moduleConstants.getEncoderPort(),
      moduleConstants.getSteerOffset(),
      moduleConstants.getDriveKS(),
      moduleConstants.getDriveKV(),
      moduleTab
    );
  }

  public ModuleSim(
    int driveMotorPort,
    int steerMotorPort,
    int encoderPort,
    double encoderOffset,
    double feedforwardKS,
    double feedforwardKV,
    ShuffleboardTab moduleTab
  ) {

    // TODO: clean constudctor
    super(driveMotorPort, steerMotorPort, encoderPort, encoderOffset, feedforwardKS, feedforwardKV,0,0,0,0,0,0,0,0, ModuleType.NONE, moduleTab);
    m_driveEncoderSim = new TalonEncoderSim(getDriveEncoder());
    m_encoderSim = new CANCoderSimCollection(getEncoder());

  }

  /**
   * Returns the current state of the module.
   * @return the current state of the module
   */
  @Override
  public SwerveModuleState getState() {
    return new SwerveModuleState(m_currentSpeed, new Rotation2d(m_currentSteerPositionRad));
  }

  /**
   * Updates the simulation.
   */
  @Override
  public void periodic() {
    m_currentDrivePositionMeters += m_currentSpeed * Constants.kLoopTime; 
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  @Override
  public void setDesiredState(SwerveModuleState desiredState) {
    if (Math.abs(desiredState.speedMetersPerSecond) < 0.001) {
      m_currentSpeed = 0;
      return;
    }
    // Optimize the reference state to avoid spinning further than 90 degrees
    desiredState = SwerveModuleState.optimize(desiredState, new Rotation2d(m_currentSteerPositionRad));

    m_currentSpeed = desiredState.speedMetersPerSecond;
    m_currentSteerPositionRad = desiredState.angle.getRadians();
  }

  /**
   * Gets the simulated angle of the module.
   */
  @Override
  public double getSteerAngle() {
    return m_currentSteerPositionRad;
  }

  @Override
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
      m_currentDrivePositionMeters,
      new Rotation2d(m_currentSteerPositionRad)
    );
  }
}