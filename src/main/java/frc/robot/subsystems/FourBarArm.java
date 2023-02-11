package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ArmConstants;

public class FourBarArm extends SubsystemBase {
  private final CANSparkMax m_motor;
  private final PIDController m_pid;
  private final RelativeEncoder m_encoder;
  private double m_armSetpoint;

  public FourBarArm() {
    m_motor = new CANSparkMax(ArmConstants.motorID, MotorType.kBrushless);
    m_motor.setIdleMode(IdleMode.kBrake);
    m_encoder = m_motor.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 8192);
    m_encoder.setPositionConversionFactor(2*Math.PI);
    m_encoder.setVelocityConversionFactor(2*Math.PI/60);  
    m_pid = new PIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD);
    m_pid.setSetpoint(ArmConstants.initialPosition);
    m_pid.setTolerance(ArmConstants.kTolerance);
  }

  public void setArmSetpoint(double setpoint) {
    m_pid.reset();
    m_pid.setSetpoint(setpoint);
    m_armSetpoint = setpoint;
  }

  @Override
  public void periodic() {
    m_motor.set(MathUtil.clamp(m_pid.calculate(m_encoder.getPosition()), ArmConstants.minMotorPower, ArmConstants.maxMotorPower));
  }

  public boolean isFinished() {
    return m_pid.atSetpoint();
  }

  public void end() {
    m_motor.set(0);
  }
}
