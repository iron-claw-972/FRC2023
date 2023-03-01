package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.controls.Operator;
import frc.robot.subsystems.Drivetrain;

/**
 * Uses PID to make the robot go to the selected pose
 */
public class MoveToSelectedPose extends CommandBase {

  private final Drivetrain m_drive;
  private final Operator m_Operator;
  public MoveToSelectedPose(Drivetrain drive, Operator operator) {
    m_drive = drive;    
    m_Operator=operator;
    addRequirements(drive);
  }

  @Override
  public void initialize(){
    m_drive.setAllOptimize(true);
  }
  
  @Override
  public void execute() {
    double x = m_Operator.getSelectedNode().scorePose.getX();
    double y = m_Operator.getSelectedNode().scorePose.getY();
    double rot = m_Operator.getSelectedNode().scorePose.getRotation().getRadians();
    m_drive.runChassisPID(x, y, rot);
  }
  
  @Override
  public void end(boolean interrupted) {
    m_drive.stop();
  }
}
