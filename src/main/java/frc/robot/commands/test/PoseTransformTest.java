package frc.robot.commands.test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.TestConstants;
import frc.robot.subsystems.Drivetrain;

/**
 * Tests the odometry of the robot by driving a certain distance and calculating the error.
 */
public class PoseTransformTest extends CommandBase {

  private Drivetrain m_drive; 
  
  private double m_startTime;
  private Pose2d m_finalPose;
  private Pose2d m_error;
  
  public PoseTransformTest(Drivetrain drive) {
    m_drive = drive; 
    // finalPose is position after robot moves from current position-- startPose-- by the values that are inputted-- distanceToMove
    
    addRequirements(drive);
  }
  
  @Override
  public void initialize() {
    m_startTime = Timer.getFPGATimestamp();
    m_finalPose = m_drive.getPose().transformBy(new Transform2d(
      new Translation2d(m_drive.getRequestedXPos().getDouble(0), m_drive.getRequestedYPos().getDouble(0)), 
      new Rotation2d(m_drive.getRequestedHeadingEntry().getDouble(0))
    ));
  }
  
  @Override
  public void execute() {
    m_drive.runChassisPID(m_finalPose.getX(), m_finalPose.getY(), m_finalPose.getRotation().getRadians()); 
  }

  @Override
  public boolean isFinished() {
    // TODO: the current PID values don't allow the command to finish
    double errorMarginMeters = TestConstants.kTranslationError;
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