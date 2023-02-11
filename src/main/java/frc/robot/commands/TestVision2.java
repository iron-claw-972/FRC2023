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
  private double encoderStart;
  private Pose2d startPose;
  private double encoderPosition;
  private Pose2d currentPose = null;
  private int endCounter = 0;
  private double m_speed;
  private int direction = 1;
  private boolean turned = false;
  private double distanceToMove;
  private double closest;

  //How many frames it has to not see anything to end the command
  private final int endDelay = 5;

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
    distanceToMove=distance;
  }

  private double getDist(){
    // return 0;
    return m_drive.getLeftDistance()/2+m_drive.getRightDistance()/2;
  }

  private Pose2d getPose(){
    Optional<Pair<Pose3d, Double>> p = Vision.getEstimatedGlobalPose(currentPose==null?m_drive.getPose():currentPose);
    if(p.isPresent() && p.get().getFirst() != null && p.get().getSecond() != null && p.get().getFirst().getX() > -10000 && p.get().getSecond() >= 0){
      return p.get().getFirst().toPose2d();
    }
    return null;
  }

  /**
   * Initializes the command
   */
  @Override
  public void initialize(){
    encoderStart=getDist();
    startPose=getPose();
    direction=1;
    turned=false;
    endCounter=0;
    closest=100;
  }

  /**
   * Moves the robot
   * If it has moves farther than the specified distance, it turns around
   * It stops when it can't see an April tag or it has passed its starting position
   */
  @Override
  public void execute(){
    m_drive.arcadeDrive(direction*m_speed, 0);
    if(getPose()==null){
      endCounter++;
    }else{
      endCounter = Math.max(0, endCounter-1);
      currentPose = getPose();
      encoderPosition = getDist();
      double dist1 = Math.abs(encoderPosition-encoderStart);
      double dist2 = Math.sqrt(Math.pow(currentPose.getX()-startPose.getX(), 2) + Math.pow(currentPose.getY()-startPose.getY(), 2));
      if(dist2>=distanceToMove&&direction==1){
        System.out.printf("Encoder distance: %.4f\n", dist1);
        direction=-1;
        turned=true;
        // endCounter=1000;
      }
      if(turned){
        closest=Math.min(dist2, closest);
        if(dist2-closest>0.05){
          System.out.println(dist2);
          System.out.println(closest);
          endCounter+=2;
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
    System.out.printf("\nVision distance: %.4f\nEncoder distance: %.4f\n", Math.sqrt(Math.pow(currentPose.getX()-startPose.getX(), 2) + Math.pow(currentPose.getY()-startPose.getY(), 2)), Math.abs(encoderPosition-encoderStart));
    m_drive.arcadeDrive(0, 0);
  }

  /**
   * Returns if the command is finished
   * @return If endCounter is greater than endDelay
   */
  @Override
  public boolean isFinished(){
    return endCounter>=endDelay;
  }
}
