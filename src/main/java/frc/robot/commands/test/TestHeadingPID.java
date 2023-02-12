package frc.robot.commands.test;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;

public class TestHeadingPID extends CommandBase {
  
  Drivetrain m_drive;
  GenericEntry m_testEntry;
  
  public TestHeadingPID(Drivetrain drive, GenericEntry testEntry) {
    m_drive = drive;
    m_testEntry = testEntry;
    addRequirements(m_drive);
  }

  @Override
  public void initialize() {
    m_drive.setAllOptimize(false);
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
    m_testEntry.setBoolean( Math.abs(m_drive.getAngleHeading()-m_drive.getRequestedHeadingEntry().getDouble(0)) < Units.degreesToRadians(2) );
  }
  
  @Override
  public void end(boolean interrupted) {
    m_drive.setAllOptimize(true);
    m_drive.stop();
  }
}
