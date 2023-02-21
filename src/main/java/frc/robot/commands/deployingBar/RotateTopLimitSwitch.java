package frc.robot.commands.deployingBar;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.DeployingBarConstants;
import frc.robot.subsystems.DeployingBar;

public class RotateTopLimitSwitch extends CommandBase {
  private final DeployingBar m_deployingBar;
  /**
   * Creates a new ExtendDeployingBar
   *
   * @param subsystem The subsystem used by this command.
   */
  public RotateTopLimitSwitch(DeployingBar deployingBar) {
    m_deployingBar = deployingBar;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(deployingBar);
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_deployingBar.setSpeed(-DeployingBarConstants.kDeployBarSpeed);
  }

  @Override
  public boolean isFinished() {
    return m_deployingBar.atTopLimitSwitch();
  }
}

