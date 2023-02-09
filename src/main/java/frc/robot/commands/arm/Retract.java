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
  PIDController m_pid;
  public Retract(FourBarArm arm, PIDController pid) {
    m_arm = arm;
    m_pid = pid; 
  }
  @Override
  public void initialize() {
    m_pid.reset();
  }
  @Override
  public void execute() {
    m_arm.setMotor(m_pid.calculate(m_arm.getEncoderPosition(), Constants.arm.initialPosition));
  }
  @Override
  public boolean isFinished() {
    double error = Constants.arm.initialPosition - m_arm.getEncoderPosition();
    return error <= -1 || error >= -1; // error values TBD
  }
  @Override
  public void end(boolean interrupted) {
    m_arm.setMotor(0);
  }
}
