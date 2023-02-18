package frc.robot.commands.test;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

/**
 * Drives robot in a circle. There is often drift due to inaccuracy.
 */
public class CircleDrive extends CommandBase {
  
  private final Drivetrain m_drive;
  private double m_steerPosition = 0, m_prevTime;
  
  public CircleDrive(Drivetrain drive) {
    m_drive = drive;
    addRequirements(m_drive);
  }
  
  @Override
  public void initialize() {
    m_prevTime = WPIUtilJNI.now() * 1e-6;
    m_steerPosition = 0;
  }
  
  @Override
  public void execute() {
    double currentTime = WPIUtilJNI.now() * 1e-6;
    m_steerPosition = MathUtil.angleModulus(m_steerPosition + (currentTime - m_prevTime) * m_drive.getRequestedSteerVelocityEntry().getDouble(0) );
    m_drive.setModuleStates(new SwerveModuleState[] {
      new SwerveModuleState(m_drive.getRequestedDriveVelocityEntry().getDouble(0),new Rotation2d(m_steerPosition)),
      new SwerveModuleState(m_drive.getRequestedDriveVelocityEntry().getDouble(0),new Rotation2d(m_steerPosition)),
      new SwerveModuleState(m_drive.getRequestedDriveVelocityEntry().getDouble(0),new Rotation2d(m_steerPosition)),
      new SwerveModuleState(m_drive.getRequestedDriveVelocityEntry().getDouble(0),new Rotation2d(m_steerPosition))
    });
    m_prevTime = currentTime;
  }
  
  @Override
  public void end(boolean interrupted) {
    m_drive.stop();
  }
}
