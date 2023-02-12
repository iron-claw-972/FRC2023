package frc.robot.commands.test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

/**
 * Tests the odometry of the robot by driving a certain distance and calculating the error.
 */
public class OdometryTestCommand extends CommandBase {

  private Drivetrain m_drive; 
  private Pose2d m_startPose;
  private double m_startTime;
  private Pose2d m_finalPose;
  private Transform2d m_distanceToMove;
  private Pose2d m_error;
  
  /**
   * Creates a new command.
   * @param drive the drivetrain instance
   */
  public OdometryTestCommand(Drivetrain drive, Transform2d distanceToMove) {
    m_drive = drive; 
    // finalPose is position after robot moves from current position-- startPose-- by the values that are inputted-- distanceToMove
    m_distanceToMove = distanceToMove;
    
    addRequirements(drive);
  }
  
  @Override
  public void initialize() {
    m_startTime = Timer.getFPGATimestamp();
    m_startPose = m_drive.getPose();
    m_finalPose = m_startPose.transformBy(m_distanceToMove);
  }
  
  @Override
  public void execute() {
    m_drive.runChassisPID(m_finalPose.getX(), m_finalPose.getY(), m_finalPose.getRotation().getRadians()); 
  }

  @Override
  public boolean isFinished() {
    // TODO: the current PID values don't allow the command to finish
    double errorMarginMeters = 0.1;
    double errorMarginRadians = Units.degreesToRadians(10);
    m_error = m_drive.getPose().relativeTo(m_finalPose);
    // if robot thinks its precision is < 0.1 to the target we inputted, it will stop, so then we can see how off it is
    return Math.abs(m_error.getX()) < errorMarginMeters && Math.abs(m_error.getY()) < errorMarginMeters && Math.abs(m_error.getRotation().getRadians()) < errorMarginRadians;
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.stop();
    System.out.println(Timer.getFPGATimestamp() - m_startTime);
    System.out.println(m_error.getX());
    System.out.println(m_error.getY());
    System.out.println(m_error.getRotation().getRadians());
  }
}

