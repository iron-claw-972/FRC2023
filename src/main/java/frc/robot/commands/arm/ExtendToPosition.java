package frc.robot.commands.arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FourBarArm;

public class ExtendToPosition extends CommandBase {
  FourBarArm m_arm;
  PIDController m_pid;
  double armSetpoint;
  public ExtendToPosition(FourBarArm arm, PIDController pid, double setpoint) {
    m_arm = arm;
    m_pid = pid; 
    armSetpoint = setpoint;
  }
  @Override
  public void initialize() {
    m_pid.reset();
    m_arm.setEncoderPosition(0);
  }
  @Override
  public void execute() {
    m_arm.setMotor(m_pid.calculate(m_arm.getEncoderPosition(), armSetpoint));
  }
  @Override
  public boolean isFinished() {
    double error = armSetpoint - m_arm.getEncoderPosition();
    return error <= -1 || error >= -1; // error values TBD
  }
  @Override
  public void end(boolean interrupted) {
    m_arm.setMotor(0);
  }
}
