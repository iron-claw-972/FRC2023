package frc.robot.commands;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

/**
 * Sets all module angles to a given trajectory's initial angle.
 */
public class ModuleAnglesFromTrajectory extends CommandBase {

  private final Drivetrain m_drive;
  private final Trajectory m_trajectory;
  
  public ModuleAnglesFromTrajectory(Drivetrain drive, Trajectory trajectory) {
    m_drive = drive;
    m_trajectory = trajectory;
    
    addRequirements(drive);
  }
  
  @Override
  public void execute() {
    m_drive.setAllOptimize(true);
    m_drive.setModuleStates(new SwerveModuleState[] {
      new SwerveModuleState(0, m_trajectory.getInitialPose().getRotation()),
      new SwerveModuleState(0, m_trajectory.getInitialPose().getRotation()),
      new SwerveModuleState(0, m_trajectory.getInitialPose().getRotation()),
      new SwerveModuleState(0, m_trajectory.getInitialPose().getRotation())
    });
  }

  @Override
  public boolean isFinished() {
    return (
      m_drive.m_modules[0].getSteerAngle() == m_trajectory.getInitialPose().getRotation().getRadians() &&
      m_drive.m_modules[1].getSteerAngle() == m_trajectory.getInitialPose().getRotation().getRadians() &&
      m_drive.m_modules[2].getSteerAngle() == m_trajectory.getInitialPose().getRotation().getRadians() &&
      m_drive.m_modules[3].getSteerAngle() == m_trajectory.getInitialPose().getRotation().getRadians()
    );
  }
  
  @Override
  public void end(boolean interrupted) {
    m_drive.stop();
  }
    
}
