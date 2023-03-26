package frc.robot.commands.vision;

import java.util.ArrayList;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.LogManager;
import frc.robot.util.Vision;

/**
 * Calculates standard deviations for vision
 */
public class CalculateStdDevs extends CommandBase {
  private final Drivetrain m_drive;
  private final Vision m_vision;
  private ArrayList<Pose2d> m_poses;
  private int m_arrayLength;
  private Timer m_endTimer;

  /**
   * Constructor for CalculateStdDevs
   * @param time How long to run the command, also how many poses to use
   * @param drive The drivetrain
   * @param vision The vision
   */
  public CalculateStdDevs(int time, Drivetrain drive, Vision vision) {
    m_drive = drive;
    m_vision = vision;
    m_arrayLength = time;
    m_endTimer = new Timer();
  }

  /**
   * Resets the pose array
   */
  @Override
  public void initialize() {
    m_poses = new ArrayList<Pose2d>();
  }

  /**
   * Adds a pose to the array
   */
  @Override
  public void execute() {
    Pose2d pose = m_vision.getPose2d(m_drive.getPose());
    // If the pose exists, add it to the first open spot in the array
    if (pose != null) {
      m_endTimer.stop();
      m_endTimer.reset();
      m_poses.add(pose);
      System.out.printf("%.1f%% done", ((double)m_poses.size())/m_arrayLength * 100);
    } else {
      m_endTimer.start();
      // If 5 seconds have passed since it saw an April tag, stop the command
      // Prevents it from running forever
      if(m_endTimer.hasElapsed(5)){
        cancel();
      }
    }
  }

  /**
   * Calculates the standard deviation
   */
  @Override
  public void end(boolean interrupted) {
    // If the array is empty, don't try to calculate std devs
    if(m_poses.size() == 0){
      System.out.println("There are no poses in the array\nTry again where the robot can see an April tag.");
      return;
    }
    
    // Calculate std devs
    double meanX = 0;
    double meanY = 0;
    double meanRot = 0;
    for (int i = 0; i < m_poses.size(); i++) {
      meanX += m_poses.get(i).getX();
      meanY += m_poses.get(i).getY();
      meanRot += m_poses.get(i).getRotation().getRadians();
    }
    meanX /= m_poses.size();
    meanY /= m_poses.size();
    meanRot /= m_poses.size();
    double totalX = 0;
    double totalY = 0;
    double totalRot = 0;
    for (int i = 0; i < m_poses.size(); i++) {
      totalX += Math.pow(m_poses.get(i).getX() - meanX, 2);
      totalY += Math.pow(m_poses.get(i).getY() - meanY, 2);
      totalRot += Math.pow(m_poses.get(i).getRotation().getRadians() - meanRot, 2);
    }
    double stdDevX = Math.sqrt(totalX / m_poses.size());
    double stdDevY = Math.sqrt(totalY / m_poses.size());
    double stdDevRot = Math.sqrt(totalRot / m_poses.size());

    /*
     * Standard deviation values:
      X: 0.21803
      Y: 0.15641
      Rotation: 0.08443
      Distance: Infinity

      deviation values:
      X: 0.00471
      Y: 0.03537
      Rotation: 0.01886
      Distance: Infinity
     */
    
    // Calculate distance to closest April tag
    double closest = 1000000;
    ArrayList<EstimatedRobotPose> estimatedPoses = m_vision.getEstimatedPoses(m_drive.getPose());
    for (int i = 0; i < estimatedPoses.size(); i++) {
      for (int j = 0; j < estimatedPoses.get(i).targetsUsed.size(); j++) {
        double distance = estimatedPoses.get(i).targetsUsed.get(j).getBestCameraToTarget()
          .getTranslation().toTranslation2d().getNorm();
        closest = Math.min(closest, distance);
      }
    }
    
    // Store data in an array to make logging and shuffleboard easier
    double[] stdDevs = {stdDevX, stdDevY, stdDevRot, closest};
    
    // Print, log, and add the values to Shuffleboard
    System.out.printf("Standard deviation values:\nX: %.5f\nY: %.5f\nRotation: %.5f\nDistance: %.5f\n",
    stdDevX, stdDevY, stdDevRot, closest);
    if (Constants.kLogging) {
      LogManager.addDoubleArray("Vision/StdDevs", stdDevs);
    }
  }

  /**
   * Returns if the command is finished
   * @return If the array is full
   */
  @Override
  public boolean isFinished() {
    return m_poses.size() == m_arrayLength;
  }
}