package frc.robot.commands.test;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.TestConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.TimeAccuracyTest;

public class TestSteerAngle extends CommandBase{
  
  private Drivetrain m_drive;
  private GenericEntry m_testEntry;
  private TimeAccuracyTest m_timeAccuracyTest;

  public TestSteerAngle(Drivetrain drive, GenericEntry testEntry) {
    m_drive = drive;
    m_testEntry = testEntry;
    addRequirements(m_drive);
  }

  @Override
  public void initialize() {
    m_drive.setAllOptimize(false);
    m_drive.enableStateDeadband(false);
    m_timeAccuracyTest = new TimeAccuracyTest(
      () -> m_drive.isDriveVelocityAccurate(),
      () -> m_drive.getRequestedSteerVelocityEntry().getDouble(0),
      TestConstants.kSteerAngleTimeError
    );
  }
  
  @Override
  public void execute() {
    m_drive.setModuleStates(new SwerveModuleState[] {
      new SwerveModuleState(0, new Rotation2d(m_drive.getRequestedSteerVelocityEntry().getDouble(0))),
      new SwerveModuleState(0, new Rotation2d(m_drive.getRequestedSteerVelocityEntry().getDouble(0))),
      new SwerveModuleState(0, new Rotation2d(m_drive.getRequestedSteerVelocityEntry().getDouble(0))),
      new SwerveModuleState(0, new Rotation2d(m_drive.getRequestedSteerVelocityEntry().getDouble(0)))
    });
    m_testEntry.setBoolean(m_timeAccuracyTest.calculate());
  }
  
  @Override
  public void end(boolean interrupted) {
    m_drive.setAllOptimize(true);
    m_drive.enableStateDeadband(true);
    m_drive.stop();
  }
  
}
