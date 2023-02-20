package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.controls.Operator;
import frc.robot.subsystems.Drivetrain;

/**
 * Uses PID to make the robot go to a specific pose
 */
public class MoveToSelectedPose extends CommandBase {

  private final Drivetrain m_drive;
  
  public MoveToSelectedPose(Drivetrain drive) {
    m_drive = drive;    
    addRequirements(drive);
  }
  
  @Override
  public void execute() {
    m_drive.setAllOptimize(true);
    
    double x = Operator.selectedNode.scorePose.getX();
    double y = Operator.selectedNode.scorePose.getY();
    double rot = Operator.selectedNode.scorePose.getRotation().getRadians();

    m_drive.runChassisPID(x, y, rot);
  }
  
  @Override
  public void end(boolean interrupted) {
    m_drive.drive(0.0, 0.0, 0.0, false);
  }
}
