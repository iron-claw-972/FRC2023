package frc.robot.commands.test;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.TestConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Module;
import frc.robot.util.TimeAccuracyTest;

/**
 * Attempts to run all four modules at constant velocity. Determines if the modules are able to reach the velocity requested in a certain time.
 */
public class TestDriveVelocity extends CommandBase{
  
  private Drivetrain m_drive;
  private GenericEntry m_testEntry;
  private TimeAccuracyTest[] m_timeAccuracyTests = new TimeAccuracyTest[4];
  
  public TestDriveVelocity(Drivetrain drive, GenericEntry testEntry) {
    m_drive = drive;
    m_testEntry = testEntry;
    addRequirements(m_drive);
  }
  
  @Override
  public void initialize() {
    m_drive.setAllOptimize(false);
    for (int i = 0; i < 4; i++){
      Module module = m_drive.m_modules[i];
      m_timeAccuracyTests[i] = new TimeAccuracyTest(
        () -> module.getDriveVelocityError(),
        () -> m_drive.getRequestedSteerVelocityEntry().getDouble(0),
        TestConstants.kDriveVelocityError,
        TestConstants.kDriveVelocityTimeError
      );
    }
  }
  
  @Override
  public void execute() {
    m_drive.setModuleStates(new SwerveModuleState[] {
      new SwerveModuleState(m_drive.getRequestedDriveVelocityEntry().getDouble(0), new Rotation2d(Units.degreesToRadians(135))),
      new SwerveModuleState(m_drive.getRequestedDriveVelocityEntry().getDouble(0), new Rotation2d(Units.degreesToRadians(45))),
      new SwerveModuleState(m_drive.getRequestedDriveVelocityEntry().getDouble(0), new Rotation2d(Units.degreesToRadians(225))),
      new SwerveModuleState(m_drive.getRequestedDriveVelocityEntry().getDouble(0), new Rotation2d(Units.degreesToRadians(315)))
    });
    m_testEntry.setBoolean(
      m_timeAccuracyTests[0].calculate() &&
      m_timeAccuracyTests[1].calculate() &&
      m_timeAccuracyTests[2].calculate() &&
      m_timeAccuracyTests[3].calculate()
    );
  }
  
  @Override
  public void end(boolean interrupted) {
    m_drive.setAllOptimize(true);
    m_drive.stop();
  }
  
}
