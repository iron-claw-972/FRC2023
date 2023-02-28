package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

/**
 * Attempts to set all four modules to a constant angle. Determines if the modules are able to reach the angle requested in a certain time.
 */
public class AlignWheelsToZero extends CommandBase{
  
  private final Drivetrain m_drive;

  public AlignWheelsToZero(Drivetrain drive) {
    m_drive = drive;
    addRequirements(m_drive);
  }

  @Override
  public void initialize() {
    m_drive.setAllOptimize(false);
    m_drive.enableStateDeadband(false);
  }
  
  @Override
  public void execute() {
    m_drive.setModuleStates(new SwerveModuleState[] {
      new SwerveModuleState(0, new Rotation2d(0)),
      new SwerveModuleState(0, new Rotation2d(0)),
      new SwerveModuleState(0, new Rotation2d(0)),
      new SwerveModuleState(0, new Rotation2d(0))
    });
  }
  
  @Override
  public void end(boolean interrupted) {
    m_drive.setAllOptimize(true);
    m_drive.enableStateDeadband(true);
    m_drive.stop();
  }
  
}
