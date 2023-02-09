package frc.robot.commands;

import frc.robot.constants.Constants;
import frc.robot.subsystems.DeployingBar;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class RetractExtendingBar extends CommandBase {
  private final DeployingBar m_deployingbar;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public RetractExtendingBar(DeployingBar subsystem) {
    m_deployingbar = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_deployingbar.toggleCoast();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_deployingbar.setSetpoint(0);
    
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
