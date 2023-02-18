package frc.robot.commands.test;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.TestConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Module;
import frc.robot.util.TimeAccuracyTest;

/**
 * Attempts to set all four modules to a constant angle. Determines if the modules are able to reach the angle requested in a certain time.
 */
public class TestSteerAngle extends CommandBase{
  
  private final Drivetrain m_drive;
  private final GenericEntry m_testEntry;
  private TimeAccuracyTest m_timeAccuracyTests[]  = new TimeAccuracyTest[4];

  public TestSteerAngle(Drivetrain drive, GenericEntry testEntry) {
    m_drive = drive;
    m_testEntry = testEntry;
    addRequirements(m_drive);
  }

  @Override
  public void initialize() {
    m_drive.setAllOptimize(false);
    m_drive.enableStateDeadband(false);
    for (int i = 0; i < 4; i++){
      Module module = m_drive.m_modules[i];
      m_timeAccuracyTests[i] = new TimeAccuracyTest(
        () -> module.getDriveVelocityError(),
        () -> m_drive.getRequestedSteerVelocityEntry().getDouble(0),
        TestConstants.kSteerAngleError,
        TestConstants.kSteerAngleTimeError
      );
    }
  }
  
  @Override
  public void execute() {
    m_drive.setModuleStates(new SwerveModuleState[] {
      new SwerveModuleState(0, new Rotation2d(m_drive.getRequestedSteerAngleEntry().getDouble(0))),
      new SwerveModuleState(0, new Rotation2d(m_drive.getRequestedSteerAngleEntry().getDouble(0))),
      new SwerveModuleState(0, new Rotation2d(m_drive.getRequestedSteerAngleEntry().getDouble(0))),
      new SwerveModuleState(0, new Rotation2d(m_drive.getRequestedSteerAngleEntry().getDouble(0)))
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
    m_drive.enableStateDeadband(true);
    m_drive.stop();
  }
  
}
