package frc.robot.commands.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.LogManager;
import frc.robot.util.Vision;

/**
 * Gathers data on the distance limits of the camera used for vision.
 */
public class TestVisionDistance extends CommandBase {
  private final Drivetrain m_drive;
  private final Vision m_vision;
  private Translation2d m_visionStartTranslation, m_driveStartTranslation;
  private Pose2d m_currentPose = null;
  private double m_driveDistance;
  private double m_visionDistance;

  private double m_speed;

  private final Timer m_endTimer = new Timer();
  private final Timer m_printTimer = new Timer();

  // How many seconds of not seeing april tag before ending the command
  private static final double kEndDelay = 0.25;

  // How many seconds between each data print
  private static final double kPrintDelay = 1;

  /**
   * Constructor for TestVisionDistance
   * @param speed What speed to move at, negative if backward
   * @param drive The drivetrain
   * @param vision The vision
   */
  public TestVisionDistance(double speed, Drivetrain drive, Vision vision){
    addRequirements(drive);
    m_drive = drive;
    m_speed = speed;
    m_vision = vision;
  }

  /**
   * Starts the timers and disables vision for odometry
   */
  @Override
  public void initialize() {

    m_endTimer.reset();
    m_printTimer.restart();

    m_drive.enableVision(false);

    m_currentPose = m_vision.getPose2d(m_drive.getPose());
    m_visionStartTranslation = m_currentPose.getTranslation();
    m_driveStartTranslation = m_drive.getPose().getTranslation();
    m_driveDistance = 0;
    m_visionDistance = 0;
  }

  /**
   * Drives the robot, finds the pose fromt he drivetrain and vision, and someimes prints the distances
   */
  @Override
  public void execute() {
    m_drive.drive(m_speed, 0, 0, false, false);
    Pose2d newestPose = m_vision.getPose2d(m_currentPose, m_drive.getPose());

    // If the camera can see the apriltag
    if (newestPose != null) {
      //update current pose
      m_currentPose = newestPose;
      // reset the timer
      m_endTimer.reset();
      // If kPrintDelay seconds have passed, print the data
      m_driveDistance = m_drive.getPose().getTranslation().getDistance(m_driveStartTranslation);
      m_visionDistance = m_currentPose.getTranslation().getDistance(m_visionStartTranslation);
      m_vision.getVisionTestDriveEntry().setDouble(m_driveDistance);
      m_vision.getVisionTestVisionEntry().setDouble(m_visionDistance);
      m_vision.getVisionTestDiffEntry().setDouble(m_visionDistance - m_driveDistance);
      m_vision.getVisionTestPercentDiffEntry().setDouble((m_visionDistance-m_driveDistance) / m_driveDistance * 100);
      if (m_printTimer.advanceIfElapsed(kPrintDelay)) {
        System.out.println(m_driveDistance + " " + m_visionDistance);
      }
      if(Constants.kLogging){
        LogManager.addDouble("Vision/Distance Test Drive Distance", m_driveDistance);
        LogManager.addDouble("Vision/Distance Test Vision Distance", m_visionDistance);
        LogManager.addDouble("Vision/Distance Test Vision Error Value", m_visionDistance - m_driveDistance);
        LogManager.addDouble("Vision/Distance Test Vision Error Percentage", (m_visionDistance - m_driveDistance) / m_driveDistance * 100);
      }
    } else {
      m_endTimer.start();
    }
  }

  /**
   * Re-enables vision and stops the robot
   */
  @Override
  public void end(boolean interrupted) {
    m_drive.enableVision(true);
    m_drive.stop();
  }

  /**
   * Returns if the command is finished
   * @return If the end delay has elapsed
   */
  @Override
  public boolean isFinished() {
    return m_endTimer.hasElapsed(kEndDelay);
  }
}