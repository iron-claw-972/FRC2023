package frc.robot.commands.test;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.TestConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.TimeAccuracyTest;

/**
 * Attempts to change the heading of the chassis. Determines if the modules are able to reach the heading requested in a certain time.
 */
public class TestHeadingPID extends CommandBase {
  
  private Drivetrain m_drive;
  private GenericEntry m_testEntry;
  private TimeAccuracyTest m_timeAccuracyTest;
  
  /**
   * Creates a new command.
   * @param drive the drivetrain instance
   */
  public TestHeadingPID(Drivetrain drive, GenericEntry testEntry) {
    m_drive = drive;
    m_testEntry = testEntry;
    addRequirements(m_drive);
  }

  @Override
  public void initialize() {
    m_drive.setAllOptimize(false);
    m_timeAccuracyTest = new TimeAccuracyTest(
      () -> m_drive.getAngleHeading(),
      () -> m_drive.getRequestedHeadingEntry().getDouble(0),
      TestConstants.kHeadingError,
      TestConstants.kHeadingTimeError
    );
  }
  
  @Override
  public void execute() {
    double headingPIDOutput = m_drive.getRotationController().calculate(m_drive.getAngleHeading(), m_drive.getRequestedHeadingEntry().getDouble(0));
    // headingOutput is in rad/s. Need to convert to m/s by multiplying by radius
    headingPIDOutput *= Math.sqrt(0.5) * DriveConstants.kTrackWidth;
    m_drive.setModuleStates(
    new SwerveModuleState[] {
      new SwerveModuleState(headingPIDOutput, new Rotation2d(Units.degreesToRadians(135))),
      new SwerveModuleState(headingPIDOutput, new Rotation2d(Units.degreesToRadians(45))),
      new SwerveModuleState(headingPIDOutput, new Rotation2d(Units.degreesToRadians(225))),
      new SwerveModuleState(headingPIDOutput, new Rotation2d(Units.degreesToRadians(315)))
    });
    m_testEntry.setBoolean(m_timeAccuracyTest.calculate());
  }
  
  @Override
  public void end(boolean interrupted) {
    m_drive.setAllOptimize(true);
    m_drive.stop();
  }
}
