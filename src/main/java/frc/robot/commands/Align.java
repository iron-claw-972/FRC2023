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
  private double setpoint;
  private double angle;
  private PIDController m_pid = new PIDController(1, 0.01, 0.1);

  public Align(double angle, Drivetrain drive){
    addRequirements(drive);
    setpoint=angle;
    m_drive=drive;
  }

  private double getAngle(){
    Optional<Pair<Pose3d, Double>> p = Vision.getEstimatedGlobalPose(m_drive.getPose());
    if(p.isPresent() && p.get().getFirst() != null && p.get().getSecond() != null && p.get().getFirst().getX() > -10000 && p.get().getSecond() >= 0){
      return p.get().getFirst().toPose2d().getRotation().getRadians();
    }
    return angle;
  }

  @Override
  public void initialize(){
    angle=setpoint-2*Math.PI;
    angle=getAngle();
  }

  @Override
  public void execute(){
    angle=getAngle();
    double a = angle;
    if(a<0&&setpoint>0){
      a+=2*Math.PI;
    }else if(a>0&&setpoint<0){
      a-=2*Math.PI;
    }
    m_drive.arcadeDrive(0, -m_pid.calculate(a, setpoint));
  }

  @Override
  public void end(boolean interrupted){
    System.out.printf("\nExact angle: %.4f degrees\n", Units.radiansToDegrees(getAngle()));
    m_drive.arcadeDrive(0, 0);
  }

  @Override
  public boolean isFinished(){
    double a = getAngle();
    if(a<0&&setpoint>0){
      a += 2*Math.PI;
    }
    return Math.abs(a-setpoint)<Units.degreesToRadians(1);
  }
}
