package frc.robot.commands.test;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class TestSteerAngle extends CommandBase{
  
  Drivetrain m_drive;
  GenericEntry m_testEntry;

  public TestSteerAngle(Drivetrain drive, GenericEntry testEntry) {
    m_drive = drive;
    m_testEntry = testEntry;
    addRequirements(m_drive);
  }
  
  @Override
  public void execute() {
    m_drive.setAllOptimize(false);
    m_drive.setModuleStates(new SwerveModuleState[] {
      new SwerveModuleState(0.01, new Rotation2d(m_drive.getRequestedSteerVelocityEntry().getDouble(0))),
      new SwerveModuleState(0.01, new Rotation2d(m_drive.getRequestedSteerVelocityEntry().getDouble(0))),
      new SwerveModuleState(0.01, new Rotation2d(m_drive.getRequestedSteerVelocityEntry().getDouble(0))),
      new SwerveModuleState(0.01, new Rotation2d(m_drive.getRequestedSteerVelocityEntry().getDouble(0)))
    });
    m_testEntry.setBoolean(m_drive.isSteerAngleAccurate());
  }
  
  @Override
  public void end(boolean interrupted) {
    m_drive.stop();
  }
  
}
