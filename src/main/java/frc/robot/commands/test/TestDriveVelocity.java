package frc.robot.commands.test;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.TestConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.TimeAccuracyTest;

/**
 * Attempts to run all four modules at constant velocity. Determines if the modules are able to reach the velocity requested in a certain time.
 */
public class TestDriveVelocity extends CommandBase{
  
  private Drivetrain m_drive;
  private GenericEntry m_testEntry;
  private TimeAccuracyTest m_timeAccuracyTest;
  
  /**
   * Creates a new command.
   * @param drive the drivetrain instance
   */
  public TestDriveVelocity(Drivetrain drive, GenericEntry testEntry) {
    m_drive = drive;
    m_testEntry = testEntry;
    addRequirements(m_drive);
  }
  
  @Override
  public void initialize() {
    m_drive.setAllOptimize(false);
    m_timeAccuracyTest = new TimeAccuracyTest(
      () -> m_drive.isDriveVelocityAccurate(), 
      () -> m_drive.getRequestedDriveVelocityEntry().getDouble(0), 
      TestConstants.kDriveVelocityTimeError
    );
    
  }
  
  @Override
  public void execute() {
    m_drive.setModuleStates(new SwerveModuleState[] {
      new SwerveModuleState(m_drive.getRequestedDriveVelocityEntry().getDouble(0), new Rotation2d(Units.degreesToRadians(135))),
      new SwerveModuleState(m_drive.getRequestedDriveVelocityEntry().getDouble(0), new Rotation2d(Units.degreesToRadians(45))),
      new SwerveModuleState(m_drive.getRequestedDriveVelocityEntry().getDouble(0), new Rotation2d(Units.degreesToRadians(225))),
      new SwerveModuleState(m_drive.getRequestedDriveVelocityEntry().getDouble(0), new Rotation2d(Units.degreesToRadians(315)))
    });
    m_testEntry.setBoolean(m_timeAccuracyTest.calculate());
  }
  
  @Override
  public void end(boolean interrupted) {
    m_drive.setAllOptimize(true);
    m_drive.stop();
  }
  
}
