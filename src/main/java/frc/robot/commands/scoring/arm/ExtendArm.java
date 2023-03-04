package frc.robot.commands.scoring.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FourBarArm;

public class ExtendArm extends CommandBase {

  private final FourBarArm m_arm;
  private double m_armSetpoint;
  
  /**
   * extends four-bar-arm to a position using PID
   */
  public ExtendArm(FourBarArm arm, double setpoint) {
    addRequirements(arm);
    m_arm = arm;
    m_armSetpoint = setpoint;
  }

  @Override
  public void initialize() {
    m_arm.setEnabled(true);
    m_arm.setArmSetpoint(m_armSetpoint);
  }

  @Override
  public boolean isFinished() {
    return m_arm.reachedSetpoint();
  }
}
