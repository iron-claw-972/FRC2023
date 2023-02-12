package frc.robot.commands.test;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class TestDriveVelocity extends CommandBase{

    private Drivetrain m_drive;;
    private GenericEntry m_testEntry;

    public TestDriveVelocity(Drivetrain drive, GenericEntry testEntry){
        m_drive = drive;
        m_testEntry = testEntry;
        addRequirements(m_drive);
    }

    @Override
    public void execute() {
        m_drive.setAllOptimize(false);
    m_drive.setModuleStates(new SwerveModuleState[] {
      new SwerveModuleState(m_drive.getRequestedDriveVelocityEntry().getDouble(0), new Rotation2d(Units.degreesToRadians(135))),
      new SwerveModuleState(m_drive.getRequestedDriveVelocityEntry().getDouble(0), new Rotation2d(Units.degreesToRadians(45))),
      new SwerveModuleState(m_drive.getRequestedDriveVelocityEntry().getDouble(0), new Rotation2d(Units.degreesToRadians(225))),
      new SwerveModuleState(m_drive.getRequestedDriveVelocityEntry().getDouble(0), new Rotation2d(Units.degreesToRadians(315)))
    });
    m_testEntry.setBoolean(m_drive.isDriveVelocityAcurate());
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.stop();
    }
    
}
