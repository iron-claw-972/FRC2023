package frc.robot.subsystems;

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

  public FourBarArm() {
    m_motor = new CANSparkMax(ArmConstants.motorID, MotorType.kBrushless);
    m_motor.setIdleMode(IdleMode.kBrake);
    m_encoder = m_motor.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 8192);
    m_pid = new PIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD);
  }

  public double getEncoderPosition() {
    return m_encoder.getPosition();
  }
  public void setEncoderPosition(double position) {
    m_encoder.setPosition(position);
  }

  public void setMotor(double val) {
    m_motor.set(val);
  }
  public double getMotorValue() {
    return m_motor.get();
  }

  public PIDController getPIDController() {
    return m_pid;
  }
}
