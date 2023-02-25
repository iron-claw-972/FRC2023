package frc.robot.commands.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.Vision;

public class VisionReturnTest extends CommandBase{
  private Drivetrain m_drive;
  private Vision m_vision;
  private Translation2d m_visionStartTranslation, m_driveStartTranslation;
  private Pose2d m_turnPose;
  private Pose2d m_currentPose = null;
  private double m_speed;
  private double m_distanceToMove;
  private int m_endCounter = 0;
  private int m_direction = 1;
  private int m_turns = 0;

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

  /**
   * Initializes the command
   */
  @Override
  public void initialize(){
    m_drive.enableVision(false);
    m_currentPose = m_vision.getPose2d(null, m_drive.getPose());
    m_visionStartTranslation = m_currentPose.getTranslation();
    m_driveStartTranslation = m_drive.getPose().getTranslation();
  }

  /**
   * Moves the robot
   * If it has moved farther than the specified distance, it moves backward
   * It then goes back to the initial pose
   */
  @Override
  public void execute(){
    m_drive.drive(m_direction * m_speed, 0, 0, false);
    Pose2d newestPose = m_vision.getPose2d(m_currentPose, m_drive.getPose());
    if(newestPose != null){
      //update current pose
      m_currentPose = newestPose;
      //reduce end counter
      m_endCounter = Math.max(0, m_endCounter - 1);


      double driveDistance = m_drive.getPose().getTranslation().getDistance(m_driveStartTranslation);
      double visionDistance = m_currentPose.getTranslation().getDistance(m_visionStartTranslation);
      
      // if it is at or past the distance shifted and the direction is greater than 0  
      if(visionDistance >= m_distanceToMove && m_direction > 0){
        System.out.printf("Encoder distance: %.4f\n", driveDistance);
        m_direction = -1;
        m_turns = 1;
        //record pose at turn
        m_turnPose = m_currentPose;
      }

      //once you have turned more than once (turn > 1)
      if(m_turns > 0){
        //distance from last recorded turn pose
        double distanceFromTurnPose = m_currentPose.getTranslation().getDistance(m_turnPose.getTranslation());
        //if the difference between distance to move and distance from turn is
        if(Math.abs(distanceFromTurnPose - m_distanceToMove)<0.1){
          m_endCounter += 2;
          m_drive.stop();
        
        // if to far from turn pose and direction negative
        }else if(distanceFromTurnPose > m_distanceToMove && m_direction < 0){
          m_direction *= -0.8;
          m_turns++;
        // if to close to turn pose and direction positive
        }else if(distanceFromTurnPose < m_distanceToMove && m_direction > 0){
          m_direction *= -0.8;
          m_turns++;
        }
        if(m_turns > 10){
          m_endCounter += 2;
        }
      }
    }else{
      m_endCounter++;
    }
  }

  /**
   * Stops the robot and prints the distance it traveled
   * @param interrupted If the command was interrupted
   */
  @Override
  public void end(boolean interrupted){
    System.out.printf("\nVision distance: %.4f\nEncoder distance: %.4f\n",
      m_drive.getPose().getTranslation().getDistance(m_driveStartTranslation),
      m_currentPose.getTranslation().getDistance(m_visionStartTranslation)
    );
    m_drive.enableVision(true);
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
