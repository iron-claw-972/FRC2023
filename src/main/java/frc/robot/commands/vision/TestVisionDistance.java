package frc.robot.commands.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.Vision;

/**
 * Gathers data on the distance limits of the camera used for vision.
 */
public class TestVisionDistance extends CommandBase{
  private final Drivetrain m_drive;
  private final Vision m_vision;
  private Translation2d m_visionStartTranslation, m_driveStartTranslation;
  private Pose2d m_currentPose = null;

  private double m_speed;

  private final Timer m_endTimer = new Timer();
  private final Timer m_printTimer = new Timer();

  // TODO: Should these values be in a constant file?
  // How many seconds of not seeing april tag before ending the command
  private static final double kEndDelay = 0.25;

  // How many seconds between each data print
  private static final double kPrintDelay = 0.25;

  public TestVisionDistance(double speed, Drivetrain drive, Vision vision){
    addRequirements(drive);
    m_drive = drive;
    m_speed = speed;
    m_vision = vision;
  }

  @Override
  public void initialize(){

    m_endTimer.reset();
    m_printTimer.restart();

    m_drive.enableVision(false);

    m_currentPose = m_vision.getPose2d(m_drive.getPose());
    m_visionStartTranslation = m_currentPose.getTranslation();
    m_driveStartTranslation = m_drive.getPose().getTranslation();
  }

  @Override
  public void execute(){
    m_drive.drive(m_speed, 0, 0, false, false);
    Pose2d newestPose = m_vision.getPose2d(m_currentPose, m_drive.getPose());

    // If the camera can see the apriltag
    if (newestPose != null){
      //update current pose
      m_currentPose = newestPose;
      // reset the timer
      m_endTimer.reset();
      // If kPrintDelay seconds have passed, print the data
      // TODO: Consider using shuffleboard instead of printing
      if (m_printTimer.advanceIfElapsed(kPrintDelay)) {
        double driveDistance = m_drive.getPose().getTranslation().getDistance(m_driveStartTranslation);
        double visionDistance = m_currentPose.getTranslation().getDistance(m_visionStartTranslation);
        System.out.printf("\nEncoder distance: %.4f\nVision distance: %.4f\n",
          driveDistance, visionDistance);
        System.out.printf("Difference: %.4f\nPercent difference: %.4f%%\n",
          visionDistance - driveDistance, (visionDistance - driveDistance) / driveDistance * 100);
      } 
    } else {
      m_endTimer.start();
    }
  }

  @Override
  public void end(boolean interrupted){
    m_drive.enableVision(true);
    m_drive.stop();
  }

  @Override
  public boolean isFinished(){
    return m_endTimer.hasElapsed(kEndDelay);
  }

}