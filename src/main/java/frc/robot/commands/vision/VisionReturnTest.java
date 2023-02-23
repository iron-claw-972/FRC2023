package frc.robot.commands.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.Vision;

public class VisionReturnTest extends CommandBase{
  private Drivetrain m_drive;
  private Vision m_vision;
  private double m_encoderStart;
  private Pose2d m_startPose;
  private double m_encoderPosition;
  private Pose2d m_currentPose = null;
  private int m_endCounter = 0;
  private double m_speed;
  private int m_direction = 1;
  private boolean m_turned = false;
  private double m_distanceToMove;

  //How many frames it has to not see anything to end the command
  private static final int endDelay = 5;

  /**
   * Moves the robot a certain distance and then moves it back
   * @param speed What speed the robot should move at
   * @param distance How far it should move
   * @param drive The drivetrain
   */
  public VisionReturnTest(double speed, double distance, Drivetrain drive, Vision vision){
    addRequirements(drive);
    m_drive=drive;
    m_vision=vision;
    m_speed=speed;
    m_distanceToMove=distance;
  }

  private double getDist(){
    return 
      m_drive.m_modules[0].getDrivePosition()/4+
      m_drive.m_modules[1].getDrivePosition()/4+
      m_drive.m_modules[2].getDrivePosition()/4+
      m_drive.m_modules[3].getDrivePosition()/4;
  }

  /**
   * Initializes the command
   */
  @Override
  public void initialize(){
    m_encoderStart=getDist();
    m_startPose=m_vision.getPose2d(m_currentPose, m_drive.getPose());
    m_direction=1;
    m_turned=false;
    m_endCounter=0;
  }

  /**
   * Moves the robot
   * If it has moved farther than the specified distance, it moves backward
   * It stops when it can't see an April tag or it has reached its starting position
   */
  @Override
  public void execute(){
    m_drive.drive(m_direction*m_speed, 0, 0, false);
    Pose2d pose = m_vision.getPose2d(m_currentPose, m_drive.getPose());
    if(pose==null){
      m_endCounter++;
    }else{
      m_endCounter = Math.max(0, m_endCounter-1);
      m_currentPose = pose;
      m_encoderPosition = getDist();
      double dist1 = Math.abs(m_encoderPosition-m_encoderStart);
      double dist2 = Math.hypot(m_currentPose.getX()-m_startPose.getX(),
        m_currentPose.getY()-m_startPose.getY());
      if(dist2>=m_distanceToMove&&m_direction==1){
        System.out.printf("Encoder distance: %.4f\n", dist1);
        m_direction=-1;
        m_turned=true;
      }
      if(m_turned && dist2 < 0.1){
          m_endCounter+=2;
          m_drive.stop();
      }
      if(m_direction == -1 && dist2 > m_distanceToMove+0.5){
        m_direction = 1;
      }
    }
  }

  /**
   * Stops the robot and prints the distance it traveled
   * @param interrupted If the command was interrupted
   */
  @Override
  public void end(boolean interrupted){
    System.out.printf("\nVision distance: %.4f\nEncoder distance: %.4f\n",
      Math.hypot(m_currentPose.getX()-m_startPose.getX(), m_currentPose.getY()-m_startPose.getY()),
      Math.abs(m_encoderPosition-m_encoderStart));
    m_drive.stop();
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
