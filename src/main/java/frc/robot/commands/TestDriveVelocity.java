package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;

public class TestDriveVelocity extends CommandBase{

    private Drivetrain m_drive;
    private double m_translationalVelocity = 0, m_rotationalVelocity = 0, m_steerPosition = 0, m_prevTime;

    TestDriveVelocity(Drivetrain drive){
        m_drive = drive;
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        m_prevTime = WPIUtilJNI.now() * 1e-6;
    }

    @Override
    public void execute() {
        m_translationalVelocity = Robot.shuffleboard.getRequestedDriveVelocity();
        m_rotationalVelocity = Robot.shuffleboard.getRequestedSteerVelocity();

        double currentTime = WPIUtilJNI.now() * 1e-6;
        m_steerPosition = MathUtil.angleModulus( m_steerPosition + (currentTime - m_prevTime) * m_rotationalVelocity );
        for (int i = 0; i < 4; i++){
            m_drive.m_modules[i].setDriveVelocity(m_translationalVelocity);
            m_drive.m_modules[i].setSteerAngle(new Rotation2d(m_steerPosition) );
        }
        m_prevTime = currentTime;
    }
    public boolean isFinished(){
        return m_translationalVelocity+0.1>Robot.drive.m_modules[0].getDriveVelocity()||
        m_translationalVelocity-0.1<Robot.drive.m_modules[0].getDriveVelocity()||
        m_translationalVelocity+0.1>Robot.drive.m_modules[1].getDriveVelocity()||
        m_translationalVelocity-0.1<Robot.drive.m_modules[1].getDriveVelocity()||
        m_translationalVelocity+0.1>Robot.drive.m_modules[2].getDriveVelocity()||
        m_translationalVelocity-0.1<Robot.drive.m_modules[2].getDriveVelocity()||
        m_translationalVelocity+0.1>Robot.drive.m_modules[3].getDriveVelocity()||
        m_translationalVelocity-0.1<Robot.drive.m_modules[3].getDriveVelocity();
    }
    @Override
    public void end(boolean interrupted) {
        for (int i = 0; i < 4; i++){
            m_drive.m_modules[i].setDriveVelocity(0);
            m_drive.m_modules[i].setSteerAngle(new Rotation2d(0) );
        }
    }
    
}
