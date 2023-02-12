package frc.robot.commands.test;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;

public class TestHeadingPID extends CommandBase {

  Drivetrain m_drive;

  // TODO: return if in error bound
  public TestHeadingPID(Drivetrain drive){
    m_drive = drive;
    addRequirements(m_drive);
  }

  @Override
  public void execute(){
    m_drive.setAllOptimize(false);
    m_drive.m_headingPIDOutput = m_drive.getRotationController().calculate(m_drive.getAngleHeading(), m_drive.getRequestedHeadingEntry().getDouble(0));
    
    // headingOutput is in rad/s. Need to convert to m/s by multiplying by radius
    m_drive.m_headingPIDOutput *= Math.sqrt(0.5) * DriveConstants.kTrackWidth;
    m_drive.setModuleStates(
     new SwerveModuleState[] {
      new SwerveModuleState(m_drive.m_headingPIDOutput, new Rotation2d(Units.degreesToRadians(135))),
      new SwerveModuleState(m_drive.m_headingPIDOutput, new Rotation2d(Units.degreesToRadians(45))),
      new SwerveModuleState(m_drive.m_headingPIDOutput, new Rotation2d(Units.degreesToRadians(225))),
      new SwerveModuleState(m_drive.m_headingPIDOutput, new Rotation2d(Units.degreesToRadians(315)))
    });
  }

  @Override
  public void end(boolean interrupted) {
      m_drive.stop();
  }
}
