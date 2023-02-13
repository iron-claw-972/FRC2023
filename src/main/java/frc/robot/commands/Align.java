package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.Vision;

public class Align extends CommandBase{
  private Drivetrain m_drive;
  private double m_setpoint;
  private double m_angle;
  // private PIDController m_pid = new PIDController(1, 0.01, 0.1);

  /**
   * Aligns the robot to a specific angle
   * @param angle The angle to go to
   * @param drive The drivetrain
   */
  public Align(double angle, Drivetrain drive){
    addRequirements(drive);
    m_setpoint=angle;
    m_drive=drive;
  }

  private double getAngle(){
    Optional<Pair<Pose3d, Double>> p = Vision.getEstimatedGlobalPose(m_drive.getPose());
    if(p.isPresent() && p.get().getFirst() != null && p.get().getSecond() != null && p.get().getFirst().getX() > -10000 && p.get().getSecond() >= 0){
      return p.get().getFirst().toPose2d().getRotation().getRadians();
    }
    return m_angle;
  }

  /**
   * Initializes the command
   */
  @Override
  public void initialize(){
    m_angle=m_setpoint-2*Math.PI;
    m_angle=getAngle();
  }

  /**
   * Turns the robot
   * PID broke the classbot, so this doesn't use it
   */
  @Override
  public void execute(){
    m_angle=getAngle();
    double a = m_angle;
    if(a<0&&m_setpoint>0){
      a+=2*Math.PI;
    }else if(a>0&&m_setpoint<0){
      a-=2*Math.PI;
    }
    double speed = a>m_setpoint?-0.1:0.1;
    m_drive.arcadeDrive(0, -speed);
  }

  /**
   * Stops the robot and prints the final angle
   * @param interrupted If the command was interrupted
   */
  @Override
  public void end(boolean interrupted){
    System.out.printf("\nExact angle: %.4f degrees\n", Units.radiansToDegrees(getAngle()));
    m_drive.arcadeDrive(0, 0);
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
