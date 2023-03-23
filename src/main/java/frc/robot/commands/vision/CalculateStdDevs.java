package frc.robot.commands.vision;

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
    // Store the length
    int length = m_poses.length;

    // If it is interrupted, copy everything into a new array
    if(interrupted){
      Pose2d[] poses = m_poses.clone();
      for(int i = 0; i < length; i++){
        if(m_poses[i]==null){
          length = i;
        }
      }
      m_poses = new Pose2d[length];
      for(int i = 0; i < length; i++){
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
    double[] stdDevs = {stdDevX, stdDevY, stdDevRot};

    // Print, log, and add the values to Shuffleboard
    System.out.printf("Standard deviation values:\nX: %.5f\nY: %.5f\nRotation: %.5f\n",
    stdDevX, stdDevY, stdDevRot);
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