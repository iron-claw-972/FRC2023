package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.BarConstants;

public class Bar extends SubsystemBase {
  private final ShuffleboardTab m_barTab;

  private final CANSparkMax m_motor;

  private final SparkMaxPIDController m_pid;
  private final RelativeEncoder m_encoder;

  private BarMode m_mode;
  private double m_desiredAngle;
  private double m_desiredPower;

  private boolean m_isCalibrated;

  public Bar(ShuffleboardTab barTab) {
    m_barTab = barTab;

    m_motor = new CANSparkMax(BarConstants.kMotorID, MotorType.kBrushless);
    m_encoder = m_motor.getEncoder();
    m_pid = m_motor.getPIDController();
    configBarMotor();

    m_mode = BarMode.UNCALIBRATED;
    m_desiredAngle = BarConstants.kStowAngle;
    m_desiredPower = 0.0;

    setupShuffleboard();
  }

  private void configBarMotor() {
    m_motor.restoreFactoryDefaults();
    m_motor.setIdleMode(IdleMode.kCoast);
    m_motor.setInverted(BarConstants.kMotorInvert);

    m_motor.enableVoltageCompensation(Constants.kRobotVoltage);

    m_motor.enableSoftLimit(SoftLimitDirection.kForward, true);
    m_motor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    m_motor.setSoftLimit(SoftLimitDirection.kForward, (float) BarConstants.kStowAngle);
    m_motor.setSoftLimit(SoftLimitDirection.kReverse, (float) BarConstants.kDeployAngle);
    
    m_encoder.setPositionConversionFactor(BarConstants.kMotorEncoderDistancePerRotation);

    m_pid.setFeedbackDevice(m_encoder);
    m_pid.setP(BarConstants.kP);
    m_pid.setI(BarConstants.kI);
    m_pid.setD(BarConstants.kD);

    m_pid.setSmartMotionMaxVelocity(BarConstants.kMaxAngularVelocity, 0);
    m_pid.setSmartMotionMaxAccel(BarConstants.kMaxAngularAccel, 0);
  }

  public void zeroEncoderAtStow() {
    m_encoder.setPosition(BarConstants.kStowAngle);
  }

  public enum BarPosition {
    STOW, DEPLOY, NONE
  }

  public enum BarMode {
    UNCALIBRATED, DISABLED, MANUAL, POSITION
  }

  public void setMode(BarMode mode) {
    if (!m_isCalibrated && !(mode == BarMode.UNCALIBRATED || mode == BarMode.DISABLED)) return;
    m_mode = mode;
  }

  public void setIdleMode(IdleMode idleMode) {
    m_motor.setIdleMode(idleMode);
  }

  public BarMode getMode() {
    return m_mode;
  }

  public void setDesiredBarPosition(BarPosition barPosition) {
    switch (barPosition) {
      case STOW:
        setDesiredAngle(BarConstants.kStowAngle);
        break;
      case DEPLOY:
        setDesiredAngle(BarConstants.kDeployAngle);
        break;
      case NONE:
        break;
    }
  }

  public BarPosition getBarPosition() {
    if (reachedDesiredAngle()) {
      if (m_desiredAngle == BarConstants.kStowAngle) {
        return BarPosition.STOW;
      } else if (m_desiredAngle == BarConstants.kDeployAngle) {
        return BarPosition.DEPLOY;
      }
      return BarPosition.NONE;
    }
    return BarPosition.NONE;
  }

  public void setDesiredAngle(double desiredAngle) {
    m_desiredAngle = desiredAngle;
  }
  
  public void setDesiredPower(double desiredPower) {
    m_desiredPower = desiredPower;
  }

  public double getAngle() {
    return m_encoder.getPosition();
  }

  public boolean reachedDesiredAngle() {
    return Math.abs(m_desiredAngle - getAngle()) < BarConstants.kAngleTolerance
          && Math.abs(m_encoder.getVelocity()) < BarConstants.kVelocityTolerance;
  }

  public void setIsCalibrated() {
    m_isCalibrated = true;
  }

  @Override
  public void periodic() {
    switch (m_mode) {
      case UNCALIBRATED:
        break;
      case DISABLED:
        if (!m_isCalibrated) break;
        m_pid.setReference(0, ControlType.kDutyCycle);
        break;
      case MANUAL:
        if (!m_isCalibrated) break;
        m_pid.setReference(m_desiredPower, ControlType.kDutyCycle);
        break;
      case POSITION:
        if (!m_isCalibrated) break;
        m_pid.setReference(m_desiredAngle, ControlType.kPosition, 0, BarConstants.kGravityCompensationFactor * Math.cos(Units.degreesToRadians(getAngle()))); // 
    }
  }

  private void setupShuffleboard() {
    if (Constants.kUseTelemetry) {
      m_barTab.addDouble("Current angle (deg)", this::getAngle);
      m_barTab.addDouble("Desired angle (deg)", () -> m_desiredAngle);
      m_barTab.addDouble("Desired Power", () -> m_desiredPower);
      m_barTab.addBoolean("Reached Desired Angle", this::reachedDesiredAngle);
      m_barTab.addDouble("Output Current (A)", m_motor::getAppliedOutput);
      m_barTab.addString("Mode", () -> m_mode.toString());
      m_barTab.addString("Bar Position", () -> getBarPosition().toString());
    }
  }
}
