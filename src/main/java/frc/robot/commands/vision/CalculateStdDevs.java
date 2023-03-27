package frc.robot.commands.vision;

import java.util.ArrayList;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.LogManager;
import frc.robot.util.StatisticsUtil;
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
  public CalculateStdDevs(int posesToUse, Drivetrain drive, Vision vision) {
    m_drive = drive;
    m_vision = vision;
    m_arrayLength = posesToUse;
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
      if(m_endTimer.hasElapsed(VisionConstants.kStdDevCommandEndTime)){
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
    double[] xArray = new double[m_poses.size()];
    double[] yArray = new double[m_poses.size()];
    double[] rotArray = new double[m_poses.size()];
    for(int i = 0; i < m_poses.size(); i++){
      xArray[i] = m_poses.get(i).getX();
      yArray[i] = m_poses.get(i).getY();
      rotArray[i] = m_poses.get(i).getRotation().getRadians();
    }
    double stdDevX = StatisticsUtil.stdDev(xArray);
    double stdDevY = StatisticsUtil.stdDev(yArray);
    double stdDevRot = StatisticsUtil.stdDev(rotArray);
    
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