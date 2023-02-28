package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

/**
 * Attempts to set all four modules to a constant angle. Determines if the modules are able to reach the angle requested in a certain time.
 */
public class SimplePresetSteerAngles extends CommandBase{
  
  private final Drivetrain m_drive;
  private Rotation2d m_rotation;

  /**
   * sets the angle of module steer to 0 to remove initial turn time and drift
   * @param drive drivetrain to be used
   */
  public SimplePresetSteerAngles(Drivetrain drive) {
    this(drive, new Rotation2d());
  }
  /**
   * sets the angle of module steer to a angle to remove initial turn time and drift
   * @param angle angle to set module steer to in radians
   * @param drive drivetrain to be used
   */
  public SimplePresetSteerAngles(Drivetrain drive, double angle) {
    this(drive, new Rotation2d(angle));
  }
  /**
   * sets the angle of module steer to a angle to remove initial turn time and drift
   * @param rotation rotation to set module steer to
   * @param drive drivetrain to be used
   */
  public SimplePresetSteerAngles(Drivetrain drive, Rotation2d rotation){
    m_drive = drive;
    m_rotation = rotation;
    addRequirements(m_drive);
  }

  @Override
  public void initialize() {
    m_drive.enableStateDeadband(false);
  }
  
  @Override
  public void execute() {
    m_drive.setModuleStates(new SwerveModuleState[] {
      new SwerveModuleState(0, m_rotation),
      new SwerveModuleState(0, m_rotation),
      new SwerveModuleState(0, m_rotation),
      new SwerveModuleState(0, m_rotation)
    });
  }
  
  @Override
  public void end(boolean interrupted) {
    m_drive.enableStateDeadband(true);
    m_drive.stop();
  }
  
}
