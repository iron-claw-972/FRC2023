package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

/**
 * Default drive command. Drives robot using driver controls.
 */
public class MoveToPose extends CommandBase {

  private final Drivetrain m_drive;
  private Pose2d m_pose;
  
  public MoveToPose(Drivetrain drive, Pose2d pose) {
    m_drive = drive;
    m_pose = pose;
    
    addRequirements(drive);
  }
  
  @Override
  public void execute() {
    m_drive.setAllOptimize(true);
    
    double x = m_pose.getX();
    double y = m_pose.getY();
    double rot = m_pose.getRotation().getRadians();

    m_drive.runChassisPID(x, y, rot);
  }
  
  @Override
  public void end(boolean interrupted) {
    m_drive.drive(0.0, 0.0, 0.0, false);
  }
}
