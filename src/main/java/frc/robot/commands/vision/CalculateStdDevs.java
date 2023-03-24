package frc.robot.commands.vision;

import java.util.ArrayList;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.LogManager;
import frc.robot.util.Vision;

/**
 * Gathers data on the distance limits of the camera used for vision.
 */
public class CalculateStdDevs extends CommandBase{
  private final ShuffleboardTab m_tab;
  private final Drivetrain m_drive;
  private final Vision m_vision;
  private Pose2d[] m_poses;

  /**
   * Constructor for CalculateStdDevs
   * @param time How long to run the command, also how many poses to use
   * @param shuffleboardTab The shuffleboard tab to output to
   * @param drive The drivetrain
   * @param vision The vision
   */
  public CalculateStdDevs(int time, ShuffleboardTab shuffleboardTab, Drivetrain drive, Vision vision){
    m_tab = shuffleboardTab;
    m_drive = drive;
    m_vision = vision;
    m_poses = new Pose2d[time];
  }

  /**
   * Resets the pose array
   */
  @Override
  public void initialize(){
    m_poses = new Pose2d[m_poses.length];
  }

  /**
   * Adds a pose to the array
   */
  @Override
  public void execute(){
    Pose2d pose = m_vision.getPose2d(m_drive.getPose());
    // If the pose exists, add it to the first open spot in the array
    if(pose!=null){
      for(int i = 0; i < m_poses.length; i++){
        if(m_poses[i] == null){
          m_poses[i] = pose;
          System.out.printf("%.1f%% done", i/m_poses.length*100);
          break;
        }
      }
    }
  }

  /**
   * Calculates the standard deviation
   */
  @Override
  public void end(boolean interrupted){
    // Store the length of the array
    int length = m_poses.length;

    // If the array is empty, don't try to calculate std devs
    if(m_poses[0]==null){
      System.out.println("There are no poses in the array\nTry again where the robot can see an April tag.");
      return;
    }

    // If it is interrupted, copy everything into a new array
    if(interrupted){
      Pose2d[] poses = m_poses.clone();
      for(int i = 0; i < length; i++){
        if(m_poses[i]==null){
          length = i;
        }
      }
      m_poses = new Pose2d[length];

      // Set the length back to the original value after using it
      length = poses.length;

      for(int i = 0; i < m_poses.length; i++){
        m_poses[i] = poses[i];
      }
    }
    
    // Calculate std devs
    double meanX = 0;
    double meanY = 0;
    double meanRot = 0;
    for(int i = 0; i < m_poses.length; i++){
      meanX+=m_poses[i].getX();
      meanY+=m_poses[i].getY();
      meanRot+=m_poses[i].getRotation().getRadians();
    }
    meanX /= m_poses.length;
    meanY /= m_poses.length;
    meanRot /= m_poses.length;
    double totalX = 0;
    double totalY = 0;
    double totalRot = 0;
    for(int i = 0; i < m_poses.length; i++){
      totalX += Math.pow(m_poses[i].getX() - meanX, 2);
      totalY += Math.pow(m_poses[i].getY() - meanY, 2);
      totalRot += Math.pow(m_poses[i].getRotation().getRadians() - meanRot, 2);
    }
    double stdDevX = Math.sqrt(totalX/m_poses.length);
    double stdDevY = Math.sqrt(totalY/m_poses.length);
    double stdDevRot = Math.sqrt(totalRot/m_poses.length);
    
    // Calculate distance to closest April tag
    double closest = Double.POSITIVE_INFINITY;
    ArrayList<EstimatedRobotPose> estimatedPoses = m_vision.getEstimatedPoses(m_drive.getPose());
    for(int i = 0; i < estimatedPoses.size(); i++){
      for(int j = 0; j < estimatedPoses.get(i).targetsUsed.size(); j++){
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
    if(Constants.kLogging){
      LogManager.addDoubleArray("Vision/StdDevs", stdDevs);
    }
    if(m_tab!=null){
      m_tab.add("Standard Deviations", stdDevs);
    }

    // Reset the length to the original value if interrupted
    m_poses = new Pose2d[length];
  }

  /**
   * Returns if the command is finished
   * @return If the array is full
   */
  @Override
  public boolean isFinished(){
    return m_poses[m_poses.length-1] != null;
  }
}