package frc.robot.commands.vision;

import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.Vision;

public class AngleAlignTest extends CommandBase{
  private Drivetrain m_drive;
  private Vision m_vision;
  private double m_setpoint;
  private double m_angle;
  // private PIDController m_pid = new PIDController(1, 0.01, 0.1);

  /**
   * Aligns the robot to a specific angle
   * @param angle The angle to go to
   * @param drive The drivetrain
   */
  public AngleAlignTest(double angle, Drivetrain drive, Vision vision){
    addRequirements(drive);
    m_setpoint = angle;
    m_drive = drive;
    m_vision = vision;
  }

  private double getAngle(){
    ArrayList<EstimatedRobotPose> p = m_vision.getEstimatedPoses(m_drive.getPose());
    Pose2d p2;
    if(p.size()==0){
      return m_angle;
    }else if(p.size()==1){
      p2 = p.get(0).estimatedPose.toPose2d();
    }else{
      p2 = new Pose2d(p.get(0).estimatedPose.getX()/2+p.get(1).estimatedPose.getX()/2, p.get(0).estimatedPose.getY()/2+p.get(1).estimatedPose.getY()/2, new Rotation2d(p.get(0).estimatedPose.toPose2d().getRotation().getRadians()/2+p.get(1).estimatedPose.toPose2d().getRotation().getRadians()/2));
    }
    return p2.getRotation().getRadians();
  }

  /**
   * Initializes the command
   */
  @Override
  public void initialize(){
    m_angle = m_setpoint-2*Math.PI;
    m_angle = getAngle();
  }

  /**
   * Turns the robot
   * PID broke the classbot, so this doesn't use it
   */
  @Override
  public void execute(){
    m_angle = getAngle();
    double a = m_angle;
    if(a<0&&m_setpoint>0){
      a += 2*Math.PI;
    }else if(a>0&&m_setpoint<0){
      a -= 2*Math.PI;
    }
    double speed = a>m_setpoint?-0.1:0.1;
    m_drive.drive(0, 0, -speed, false);
  }

  /**
   * Stops the robot and prints the final angle
   * @param interrupted If the command was interrupted
   */
  @Override
  public void end(boolean interrupted){
    System.out.printf("\nExact angle: %.4f degrees\n", Units.radiansToDegrees(getAngle()));
    m_drive.stop();
  }

  /**
   * Determines if the angle is close enough (within 1 degree)
   * @return If the command is finished
   */
  @Override
  public boolean isFinished(){
    double a = getAngle();
    if(a<0&&m_setpoint>0){
      a += 2*Math.PI;
    }
    return Math.abs(a-m_setpoint)<Units.degreesToRadians(1);
  }
}
