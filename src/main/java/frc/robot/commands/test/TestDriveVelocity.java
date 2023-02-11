package frc.robot.commands.test;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class TestDriveVelocity extends CommandBase{

    private Drivetrain m_drive;;
    private GenericEntry m_driveVelocityEntry, m_testEntry;

    public TestDriveVelocity(Drivetrain drive, GenericEntry driveVelocityEntry, GenericEntry testEntry){
        m_drive = drive;
        addRequirements(m_drive);
    }

    @Override
    public void execute() {
        m_testEntry.setBoolean(
            Math.abs(m_driveVelocityEntry.getDouble(0) - m_drive.m_modules[0].getDriveVelocity()) < 0.1 &&
            Math.abs(m_driveVelocityEntry.getDouble(0) - m_drive.m_modules[1].getDriveVelocity()) < 0.1 &&
            Math.abs(m_driveVelocityEntry.getDouble(0) - m_drive.m_modules[2].getDriveVelocity()) < 0.1 &&
            Math.abs(m_driveVelocityEntry.getDouble(0) - m_drive.m_modules[3].getDriveVelocity()) < 0.1);
    }

    @Override
    public void end(boolean interrupted) {
        for (int i = 0; i < 4; i++){
            m_drive.m_modules[i].setDriveVelocity(0);
            m_drive.m_modules[i].setSteerAngle(new Rotation2d(0) );
        }
    }
    
}
