package frc.robot.commands.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.Vision;

public class TestVisionDistance extends CommandBase{
  private Drivetrain m_drive;
  private Vision m_vision;
  private Translation2d m_visionStartTranslation, m_driveStartTranslation;
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
   * It will stop when it can't see an april tag
   * @param speed What speed to move at
   * @param drive The drivetrain
   * @param vision The vision
   */
  public TestVisionDistance(double speed, Drivetrain drive, Vision vision){
    addRequirements(drive);
    m_drive=drive;
    m_speed=speed;
    m_vision=vision;
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
   * Moves the robot and prints the distances
   * If it can't see an April tag, it increases m_endCounter
   */
  @Override
  public void execute(){
    m_drive.drive(m_speed, 0, 0, false);
    Pose2d newestPose = m_vision.getPose2d(m_currentPose, m_drive.getPose());

    if(newestPose != null){
      //update current pose
      m_currentPose = newestPose;
      // reset command end counter
      m_endCounter = 0;
      // print every few cycle
      m_printCounter++;
      if(m_printCounter % printDelay == 0){
        double driveDistance = m_drive.getPose().getTranslation().getDistance(m_driveStartTranslation);
        double visionDistance = m_currentPose.getTranslation().getDistance(m_visionStartTranslation);
        System.out.printf("\nEncoder distance: %.4f\nVision distance: %.4f\n",
          driveDistance, visionDistance);
        System.out.printf("Difference: %.4f\nPercent difference: %.4f%%\n",
          visionDistance - driveDistance, (visionDistance - driveDistance) / driveDistance * 100);
      } 
    }else{
      m_endCounter++;
    }
  }

  /**
   * Stops the robot
   * @param interrupted If the command is interrupted
  */
  @Override
  public void end(boolean interrupted){
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