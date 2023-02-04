package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.controls.Driver;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.TestType;


public class OdometryTestCommand extends CommandBase{
    Drivetrain m_drive; 
    Pose2d startPose;
    double startTime;
   Pose2d m_finalPose;
   Transform2d m_distanceToMove;
    
    public OdometryTestCommand(Drivetrain drive, Transform2d distanceToMove) {
        m_drive = drive; 
      
       // finalPose is position after robot moves from current position-- startPose-- by the values that are inputted-- distanceToMove
       m_distanceToMove = distanceToMove;
        
        addRequirements(drive);

    }

    @Override
    public void initialize() {
       
        startTime = Timer.getFPGATimestamp();
        startPose = m_drive.getPose();
        m_finalPose = startPose.transformBy(m_distanceToMove);
    }
    @Override
    public void execute() {

      m_drive.runChassisPID(m_finalPose.getX(), m_finalPose.getY(), m_finalPose.getRotation().getRadians()); 
       
    }
    @Override
    public boolean isFinished() {
        double errorMargin = 0.1;
        Pose2d error = m_drive.getPose().relativeTo(m_finalPose);
        return error.getX() < errorMargin && error.getY() < errorMargin && error.getRotation().getRadians() < errorMargin;
    // if robot thinks its precision is < 0.1 to the target we inputted, it will stop, so then we can see how off it is
    }
    @Override
  public void end(boolean interrupted) {
    m_drive.driveRot(0.0,0.0,0.0,false);
    System.out.println(Timer.getFPGATimestamp() - startTime);
    System.out.println(m_finalPose.getX());
    System.out.println(m_finalPose.getY());
    System.out.println(m_finalPose.getRotation().getRadians());
  
  }

  
}

