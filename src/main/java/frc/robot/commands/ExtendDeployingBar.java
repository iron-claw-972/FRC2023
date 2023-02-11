package frc.robot.commands;

import frc.robot.subsystems.DeployingBar;

import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class ExtendDeployingBar extends CommandBase {
  private final DeployingBar m_deployingbar;
  double setpoint;
  /**
   * Creates a new ExtendDeployingBar
   *
   * @param subsystem The subsystem used by this command.
   */
  public ExtendDeployingBar(DeployingBar deployingbar, double setpoint) {
    m_deployingbar = deployingbar;
    this.setpoint = setpoint;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(deployingbar);
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_deployingbar.zeroEncoders();
    m_deployingbar.resetPID();
    m_deployingbar.setSetpoint(setpoint);
    m_deployingbar.setEnableStatus(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_deployingbar.atSetpoint();
  }
}
