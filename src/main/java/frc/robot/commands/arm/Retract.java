package frc.robot.commands.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.FourBarArm;

public class Retract extends CommandBase {
  FourBarArm m_arm;
  CANSparkMax m_motor;
  PIDController m_pid;
  RelativeEncoder m_encoder;
  public Retract(FourBarArm arm, CANSparkMax motor, PIDController pid, RelativeEncoder encoder) {
    m_arm = arm;
    m_motor = motor;
    m_pid = pid; 
    m_encoder = encoder;
  }
  @Override
  public void initialize() {
    m_pid.reset();
  }
  @Override
  public void execute() {
    m_motor.set(m_pid.calculate(m_encoder.getPosition(), Constants.arm.initialPosition));
  }
  @Override
  public boolean isFinished() {
    double error = Constants.arm.initialPosition - m_encoder.getPosition();
    return error <= -1 || error >= -1; // error values TBD
  }
  @Override
  public void end(boolean interrupted) {
    m_motor.set(0);
    m_motor.setIdleMode(IdleMode.kBrake);
  }
}
