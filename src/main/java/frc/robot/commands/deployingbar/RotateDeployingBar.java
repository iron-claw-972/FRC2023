package frc.robot.commands.deployingbar;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DeployingBar;

public class RotateDeployingBar extends CommandBase {
  private final DeployingBar m_deployingBar;
  private double m_setpoint;
  
  /**
   * Rotates the deploying bar to a radian position using PID
   *
   * @param deployingBar The subsystem used by this command.
   * @param setpoint The target location in radians.
   */
  public RotateDeployingBar(DeployingBar deployingBar, double setpoint) {
    m_deployingBar = deployingBar;
    this.m_setpoint = setpoint;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(deployingBar);
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_deployingBar.setSetpoint(m_setpoint);
    m_deployingBar.setEnable(true);
  }

  @Override
  public void end(boolean interrupted) {
    m_deployingBar.setEnable(false);
  }
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_deployingBar.atSetpoint();
  }
}
