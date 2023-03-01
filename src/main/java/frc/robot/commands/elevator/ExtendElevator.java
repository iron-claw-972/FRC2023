package frc.robot.commands.elevator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class ExtendElevator extends CommandBase {
  
  private final Elevator m_elevator; 
  private final double m_targetExtension; 
  
  /**
   * Moves the elevator to a specific extension point.
   * 
   * @param elevator the elevator subsystem to use
   * @param targetExtension the amount the elevator should extend, in meters. This is the diagonal distance, not vertical.
   */
  public ExtendElevator(Elevator elevator, double targetExtension) {
    m_elevator = elevator; 
    m_targetExtension = targetExtension; 
    addRequirements(m_elevator);
  }

  @Override
  public void initialize() {
    m_elevator.setTargetExtension(m_targetExtension); 
    m_elevator.setPIDEnabled(true);
  }
  
  @Override
  public boolean isFinished() {
    return m_elevator.atSetpoint();
  }
}
