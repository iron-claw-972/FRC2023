package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.controls.Operator;
import frc.robot.subsystems.Drivetrain;

/**
 * Uses PID to make the robot go to the selected pose
 */
public class MoveToSelectedPose extends CommandBase {

  private final Drivetrain m_drive;
  
  public MoveToSelectedPose(Drivetrain drive) {
    m_drive = drive;    
    addRequirements(drive);
  }

  @Override
  public void initialize(){
    m_drive.setAllOptimize(true);
  }
  
  @Override
  public void execute() {
    double x = Operator.getSelectedNode().scorePose.getX();
    double y = Operator.getSelectedNode().scorePose.getY();
    double rot = Operator.getSelectedNode().scorePose.getRotation().getRadians();
    m_drive.runChassisPID(x, y, rot);
  }
  
  @Override
  public void end(boolean interrupted) {
    m_drive.stop();
  }
}