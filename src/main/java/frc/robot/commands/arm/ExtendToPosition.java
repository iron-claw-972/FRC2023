package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FourBarArm;

public class ExtendToPosition extends CommandBase {
  FourBarArm m_arm;
  double m_armSetpoint;

  public ExtendToPosition(FourBarArm arm, double setpoint) {
    addRequirements(arm);
    m_arm = arm;
    m_armSetpoint = setpoint;
  }

  @Override
  public void initialize() {
    m_arm.setArmSetpoint(m_armSetpoint);
  }

  @Override
  public boolean isFinished() {
    return m_arm.reachedSetpoint();
  }
}
