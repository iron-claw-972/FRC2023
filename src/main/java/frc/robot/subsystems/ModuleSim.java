package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.constants.Constants;
import frc.robot.constants.swerve.ModuleConstants;

/**
 * Swerve module for drivetrain to be used inside of simulation.
 * 
 * @see frc.robot.subsystems.Module
 */
public class ModuleSim extends Module {

  // private final FlywheelSim m_driveMotorSim = new FlywheelSim(DCMotor.getFalcon500(1), DriveConstants.kConstants.kDriveGearRatio, 0.025);
  // private final FlywheelSim m_steerMotorSim = new FlywheelSim(DCMotor.getFalcon500(1), DriveConstants.kConstants.kSteerGearRatio, 0.004096955);


  private double m_currentSteerPositionRad = 0;
  private double m_currentDrivePositionMeters = 0;
  private double m_currentSpeed = 0;

  public ModuleSim(ModuleConstants moduleConstants, ShuffleboardTab moduleTab) {
    super(
      moduleConstants, moduleTab
    );
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