package frc.robot.commands.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FourBarArm;

public class ExtendToPosition extends CommandBase {
  FourBarArm m_arm;
  CANSparkMax m_motor;
  PIDController m_pid;
  RelativeEncoder m_encoder;
  double armSetpoint;
  public ExtendToPosition(FourBarArm arm, CANSparkMax motor, PIDController pid, RelativeEncoder encoder, double setpoint) {
    m_arm = arm;
    m_motor = motor;
    m_pid = pid; 
    m_encoder = encoder;
    armSetpoint = setpoint;
  }
  @Override
  public void initialize() {
    m_pid.reset();
    m_encoder.setPosition(0);
  }
  @Override
  public void execute() {
    m_motor.set(m_pid.calculate(m_encoder.getPosition(), armSetpoint));
  }
  @Override
  public boolean isFinished() {
    double error = armSetpoint - m_encoder.getPosition();
    return error <= -1 || error >= -1; // error values TBD
  }
  @Override
  public void end(boolean interrupted) {
    m_motor.set(0);
    m_motor.setIdleMode(IdleMode.kBrake);
  }
}
