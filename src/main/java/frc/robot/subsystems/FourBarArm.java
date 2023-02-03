package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class FourBarArm extends SubsystemBase {
  private final CANSparkMax m_motor;
  private final SparkMaxPIDController m_pid;
  private double armSetpoint = Constants.arm.initialPosition;
  public FourBarArm() {
    m_motor = new CANSparkMax(Constants.arm.motorID, MotorType.kBrushless);
    m_pid = m_motor.getPIDController();
    m_pid.setP(Constants.arm.kP);
    m_pid.setI(Constants.arm.kI);
    m_pid.setD(Constants.arm.kD);
  }
  public void setSetpoint(double target) {
    armSetpoint = target;
  }
  @Override
  public void periodic() {
    m_pid.setReference(armSetpoint, CANSparkMax.ControlType.kPosition);
  }
}
