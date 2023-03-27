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
  private ArrayList<Pose2d> m_poses1;
  private ArrayList<Pose2d> m_poses2;
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
    m_poses1 = new ArrayList<Pose2d>();
    m_poses2 = new ArrayList<Pose2d>();
  }

  /**
   * Adds a pose to the array
   */
  @Override
  public void execute() {
    ArrayList<EstimatedRobotPose> poses = m_vision.getEstimatedPoses(m_drive.getPose());
    // If both cameras see an April tag, add the poses to the arrays
    if (poses.size() == 2) {
      m_endTimer.stop();
      m_endTimer.reset();
      m_poses1.add(poses.get(0).estimatedPose.toPose2d());
      m_poses2.add(poses.get(1).estimatedPose.toPose2d());
      System.out.printf("%.1f%% done", ((double)m_poses1.size())/m_arrayLength * 100);
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
    // If the arrays are empty, don't try to calculate std devs
    if(m_poses1.size() == 0){
      System.out.println("There are no poses in the arrays\nTry again where the robot can see an April tag.");
      return;
    }
    
    // Calculate std devs
    double[] xArray1 = new double[m_poses1.size()];
    double[] yArray1 = new double[m_poses1.size()];
    double[] rotArray1 = new double[m_poses1.size()];
    double[] xArray2 = new double[m_poses1.size()];
    double[] yArray2 = new double[m_poses1.size()];
    double[] rotArray2 = new double[m_poses1.size()];
    for(int i = 0; i < m_poses1.size(); i++){
      xArray1[i] = m_poses1.get(i).getX();
      yArray1[i] = m_poses1.get(i).getY();
      rotArray1[i] = m_poses1.get(i).getRotation().getRadians();
      xArray2[i] = m_poses2.get(i).getX();
      yArray2[i] = m_poses2.get(i).getY();
      rotArray2[i] = m_poses2.get(i).getRotation().getRadians();
    }
    double stdDevX1 = StatisticsUtil.stdDev(xArray1);
    double stdDevY1 = StatisticsUtil.stdDev(yArray1);
    double stdDevRot1 = StatisticsUtil.stdDev(rotArray1);
    double stdDevX2 = StatisticsUtil.stdDev(xArray2);
    double stdDevY2 = StatisticsUtil.stdDev(yArray2);
    double stdDevRot2 = StatisticsUtil.stdDev(rotArray2);
    
    // Calculate distance to closest April tag
    double closest1 = 1000000;
    double closest2 = 1000000;
    ArrayList<EstimatedRobotPose> estimatedPoses = m_vision.getEstimatedPoses(m_drive.getPose());
    if(estimatedPoses.size() > 0) {
      for (int i = 0; i < estimatedPoses.get(0).targetsUsed.size(); i++) {
        double distance = estimatedPoses.get(0).targetsUsed.get(i).getBestCameraToTarget()
        .getTranslation().toTranslation2d().getNorm();
        closest1 = Math.min(closest1, distance);
      }
    }
    if(estimatedPoses.size() > 1) {
      for (int i = 0; i < estimatedPoses.get(1).targetsUsed.size(); i++) {
        double distance = estimatedPoses.get(1).targetsUsed.get(i).getBestCameraToTarget()
          .getTranslation().toTranslation2d().getNorm();
        closest2 = Math.min(closest2, distance);
      }
    }
    
    // Camera names
    String name1 = m_vision.getCamera(0).getName();
    String name2 = m_vision.getCamera(1).getName();

    // Store data in an array to make logging and shuffleboard easier
    double[] stdDevs1 = {stdDevX1, stdDevY1, stdDevRot1, closest1};
    double[] stdDevs2 = {stdDevX2, stdDevY2, stdDevRot2, closest2};
    
    // Print, log, and add the values to Shuffleboard
    System.out.println("\n\nStandard deviation values:");
    System.out.printf("%s:\nX: %.5f\nY: %.5f\nRotation: %.5f\nDistance: %.5f\n",
      name1, stdDevX1, stdDevY1, stdDevRot1, closest1);
    System.out.printf("%s:\nX: %.5f\nY: %.5f\nRotation: %.5f\nDistance: %.5f\n",
      name2, stdDevX2, stdDevY2, stdDevRot2, closest2);
    if (Constants.kLogging) {
      LogManager.addDoubleArray("Vision/StdDevs/"+name1, stdDevs1);
      LogManager.addDoubleArray("Vision/StdDevs/"+name2, stdDevs2);
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