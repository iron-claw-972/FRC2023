package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.controls.Operator;
import frc.robot.subsystems.Drivetrain;

/**
 * Uses PID to make the robot go to the selected pose
 */
public class GoToNodePID extends CommandBase {

  private final Drivetrain m_drive;
  private final Operator m_operator;
  public GoToNodePID(Operator operator, Drivetrain drive) {
    m_drive = drive;    
    m_operator=operator;
    addRequirements(drive);
  }

  @Override
  public void initialize(){
    m_drive.setAllOptimize(true);
    m_drive.getXController().reset();
    m_drive.getYController().reset();
    m_drive.getRotationController().reset();
  }
  
  @Override
  public void execute() {
    double x = m_operator.getSelectedNode().scorePose.getX();
    double y = m_operator.getSelectedNode().scorePose.getY();
    double rot = m_operator.getSelectedNode().scorePose.getRotation().getRadians();
    m_drive.runChassisPID(x, y, rot);
  }
  
  @Override
  public void end(boolean interrupted) {
    m_drive.stop();
  }

  @Override
  public boolean isFinished(){
    return m_operator.getSelectedNode().scorePose.getTranslation().getDistance(
      m_drive.getPose().getTranslation()) < 0.01;
  }
}
