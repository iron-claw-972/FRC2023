package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.DeployingBarConstants;
import frc.robot.subsystems.DeployingBar;

public class CalibrateEncoder extends CommandBase {
  private final DeployingBar m_deployingBar;
  /**
   * Creates a new ExtendDeployingBar
   *
   * @param subsystem The subsystem used by this command.
   */
  public CalibrateEncoder(DeployingBar deployingBar) {
    m_deployingBar = deployingBar;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(deployingBar);
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_deployingBar.setSpeed(DeployingBarConstants.kCalibrateSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    m_deployingBar.calibrateEncoder();
  }
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_deployingBar.atBottomLimitSwitch();
  }
}
