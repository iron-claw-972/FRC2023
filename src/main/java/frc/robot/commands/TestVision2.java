package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.Vision;

public class TestVision2 extends CommandBase{
  private Drivetrain m_drive;
  private double m_encoderStart;
  private Pose2d m_startPose;
  private double m_encoderPosition;
  private Pose2d m_currentPose = null;
  private int m_endCounter = 0;
  private double m_speed;
  private int m_direction = 1;
  private boolean m_turned = false;
  private double m_distanceToMove;
  private double m_closest;

  //How many frames it has to not see anything to end the command
  private static final int endDelay = 5;

  /**
   * Moves the robot a certain distance and then moves it back
   * @param speed What speed the robot should move at
   * @param distance How far it should move
   * @param drive The drivetrain
   */
  public TestVision2(double speed, double distance, Drivetrain drive){
    addRequirements(drive);
    m_drive=drive;
    m_speed=speed;
    m_distanceToMove=distance;
  }

  private double getDist(){
    // return 0;
    return m_drive.getLeftDistance()/2+m_drive.getRightDistance()/2;
  }

  /**
   * Initializes the command
   */
  @Override
  public void initialize(){
    m_encoderStart=getDist();
    m_startPose=Vision.getPose2d(m_currentPose);
    m_direction=1;
    m_turned=false;
    m_endCounter=0;
    m_closest=100;
  }

  /**
   * Moves the robot
   * If it has moves farther than the specified distance, it turns around
   * It stops when it can't see an April tag or it has passed its starting position
   */
  @Override
  public void execute(){
    m_drive.arcadeDrive(m_direction*m_speed, 0);
    if(Vision.getPose2d(m_currentPose)==null){
      m_endCounter++;
    }else{
      m_endCounter = Math.max(0, m_endCounter-1);
      m_currentPose = Vision.getPose2d(m_currentPose);
      m_encoderPosition = getDist();
      double dist1 = Math.abs(m_encoderPosition-m_encoderStart);
      double dist2 = Math.sqrt(Math.pow(m_currentPose.getX()-m_startPose.getX(), 2) + Math.pow(m_currentPose.getY()-m_startPose.getY(), 2));
      if(dist2>=m_distanceToMove&&m_direction==1){
        System.out.printf("Encoder distance: %.4f\n", dist1);
        m_direction=-1;
        m_turned=true;
        // m_endCounter=1000;
      }
      if(m_turned){
        m_closest=Math.min(dist2, m_closest);
        if(dist2-m_closest>0.05){
          System.out.println(dist2);
          System.out.println(m_closest);
          m_endCounter+=2;
          m_drive.arcadeDrive(0, 0);
        }
      }
    }
  }

  /**
   * Stops the robot and prints the distance it traveled
   * @param interrupted If the command was interrupted
   */
  @Override
  public void end(boolean interrupted){
    System.out.printf("\nVision distance: %.4f\nEncoder distance: %.4f\n", Math.sqrt(Math.pow(m_currentPose.getX()-m_startPose.getX(), 2) + Math.pow(m_currentPose.getY()-m_startPose.getY(), 2)), Math.abs(m_encoderPosition-m_encoderStart));
    m_drive.arcadeDrive(0, 0);
  }

  /**
   * Returns if the command is finished
   * @return If m_endCounter is greater than endDelay
   */
  @Override
  public boolean isFinished(){
    return m_endCounter>=endDelay;
  }
}
