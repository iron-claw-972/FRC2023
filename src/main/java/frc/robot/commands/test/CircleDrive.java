package frc.robot.commands.test;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class CircleDrive extends CommandBase{

    private Drivetrain m_drive;
    private GenericEntry m_driveVelocityEntry;
    private GenericEntry m_steerVelocityEntry;
    private double m_steerPosition = 0, m_prevTime;

    public CircleDrive(Drivetrain drive, GenericEntry driveVelocityEntry, GenericEntry steerVelocityEntry){
        m_driveVelocityEntry =  driveVelocityEntry;
        m_steerVelocityEntry =  steerVelocityEntry;
        m_drive = drive;
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        m_prevTime = WPIUtilJNI.now() * 1e-6;
    }

    @Override
    public void execute() {

        double currentTime = WPIUtilJNI.now() * 1e-6;
        m_steerPosition = MathUtil.angleModulus( m_steerPosition + (currentTime - m_prevTime) * m_steerVelocityEntry.getDouble(0) );
        m_drive.setModuleStates( new SwerveModuleState[]{
            new SwerveModuleState(m_driveVelocityEntry.getDouble(0),new Rotation2d(m_steerPosition)),
            new SwerveModuleState(m_driveVelocityEntry.getDouble(0),new Rotation2d(m_steerPosition)),
            new SwerveModuleState(m_driveVelocityEntry.getDouble(0),new Rotation2d(m_steerPosition)),
            new SwerveModuleState(m_driveVelocityEntry.getDouble(0),new Rotation2d(m_steerPosition))
        });
        m_prevTime = currentTime;
    }
  
    @Override
    public void end(boolean interrupted) {
        m_drive.stop();
    }
    
}
