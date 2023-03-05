package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.controls.Operator;
import frc.robot.subsystems.Drivetrain;

public class GoToNodePID extends CommandBase {
  
  private Drivetrain m_drive;
  private Operator m_operator;

  public GoToNodePID(Operator operator, Drivetrain drive) {
    addRequirements(drive);
    m_drive = drive;
    m_operator = operator;
  }

  /**
   * Resets the PID
   */
  @Override
  public void initialize(){
    m_drive.getXController().reset();
    m_drive.getYController().reset();
    m_drive.getRotationController().reset();
  }

  /**
   * Runs the PID to go to the selected pose
   */
  @Override
  public void execute(){
    Pose2d pose = m_operator.getSelectedNode().scorePose;
    m_drive.runChassisPID(pose.getX(), pose.getY(), pose.getRotation().getRadians());
  }

  /**
   * Stops the robot
   * @param interrupted If the command is interrupted
   */
  @Override
  public void end(boolean interrupted){
    m_drive.stop();
  }

  @Override
  public boolean isFinished(){
    return m_drive.getXController().atSetpoint() &&
      m_drive.getYController().atSetpoint() &&
      m_drive.getRotationController().atSetpoint();
  }
}
