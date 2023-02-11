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
  private double encoderStart;
  private Pose2d startPose;
  private double encoderPosition;
  private Pose2d currentPose = null;
  private int endCounter = 0;
  private int printCounter=0;
  private double m_speed;

  //How many frames it has to not see anything to end the command
  private final int endDelay = 5;

  //How many frames it will wait between prints
  private final int printDelay = 50;

  public TestVision(double speed){
    this(speed, Robot.drive);
  }

  public TestVision(double speed, Drivetrain drive){
    addRequirements(drive);
    m_drive=drive;
    m_speed=speed;
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

  @Override
  public void initialize(){
    encoderStart=getDist();
    startPose=getPose();
  }

  @Override
  public void execute(){
    m_drive.arcadeDrive(m_speed, 0);
    if(getPose()==null){
      endCounter++;
    }else{
      endCounter = 0;
      printCounter++;
      currentPose = getPose();
      encoderPosition = getDist();
      if(printCounter%printDelay==0){
        double dist1 = Math.abs(encoderPosition-encoderStart);
        double dist2 = Math.sqrt(Math.pow(currentPose.getX()-startPose.getX(), 2) + Math.pow(currentPose.getY()-startPose.getY(), 2));
        System.out.printf("\nEncoder distance: %.4f\nVision distance: %.4f\nDifference: %.4f\nPercent difference: %.4f%%\n", dist1, dist2, dist2-dist1, (dist2-dist1)/dist1*100);
      }
    }
  }

  @Override
  public void end(boolean interrupted){
    m_drive.arcadeDrive(0, 0);
  }

  @Override
  public boolean isFinished(){
    return endCounter>=endDelay;
  }
}
