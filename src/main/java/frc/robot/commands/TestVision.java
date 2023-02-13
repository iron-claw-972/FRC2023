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

public class TestVision extends CommandBase{
  private Drivetrain m_drive;
  private double m_encoderStart;
  private Pose2d m_startPose;
  private double m_encoderPosition;
  private Pose2d m_currentPose = null;
  private int m_endCounter = 0;
  private int m_printCounter=0;
  private double m_speed;

  //How many frames it has to not see anything to end the command
  private static final int endDelay = 5;

  //How many frames it will wait between prints
  private static final int printDelay = 50;

  /**
   * A command that moves and prints out distances
   * @param speed What speed to move at
   * @param drive The drivetrain
   */
  public TestVision(double speed, Drivetrain drive){
    addRequirements(drive);
    m_drive=drive;
    m_speed=speed;
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
  }

  /**
   * Moves the robot and prints the distances
   * If it can't see an April tag, it increases m_endCounter
   */
  @Override
  public void execute(){
    m_drive.arcadeDrive(m_speed, 0);
    if(Vision.getPose2d(m_currentPose)==null){
      m_endCounter++;
    }else{
      m_endCounter = 0;
      m_printCounter++;
      m_currentPose = Vision.getPose2d(m_currentPose);
      m_encoderPosition = getDist();
      if(m_printCounter%printDelay==0){
        double dist1 = Math.abs(m_encoderPosition-m_encoderStart);
        double dist2 = Math.sqrt(Math.pow(m_currentPose.getX()-m_startPose.getX(), 2) + Math.pow(m_currentPose.getY()-m_startPose.getY(), 2));
        System.out.printf("\nEncoder distance: %.4f\nVision distance: %.4f\nDifference: %.4f\nPercent difference: %.4f%%\n", dist1, dist2, dist2-dist1, (dist2-dist1)/dist1*100);
      }
    }
  }

  /**
   * Stops the robot
   * @param interrupted If the command is interrupted
  */
  @Override
  public void end(boolean interrupted){
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
