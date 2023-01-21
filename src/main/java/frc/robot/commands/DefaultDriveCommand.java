package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.controls.Driver;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.TestType;

public class DefaultDriveCommand extends CommandBase {
  private final Drivetrain m_drive;

  public DefaultDriveCommand(Drivetrain drive) {
    this.m_drive = drive;

    addRequirements(drive);
  }

  @Override
  public void execute() {

    if (Robot.shuffleboard.getTestModeType() == TestType.TUNE_HEADING_PID) {
      runHeadingPID();
      return;
    } else if (Robot.shuffleboard.getTestModeType() == TestType.TUNE_MODULE_DRIVE) {
      testDriveVel();
      Robot.shuffleboard.getModulefeedforward();
      return;
    } else if (Robot.shuffleboard.getTestModeType() == TestType.TUNE_MODULE_TURN){
      testSteerAngle();
      return;
    } else if (Robot.shuffleboard.getTestModeType() == TestType.DRIVE_VOLTAGE){
      testDriveVolts();
    return;
    } else if (Robot.shuffleboard.getTestModeType() == TestType.STEER_VOLTAGE){
      testSteerVolts();
    return;
    }

    double xSpeed = Driver.getForwardTranslation();
    double ySpeed = Driver.getSideTranslation();
    double rot = Driver.getRotation();

    m_drive.drive(xSpeed, ySpeed, rot, true);
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.drive(0.0, 0.0, 0.0, false);
  }

  private void runHeadingPID() {
    m_drive.m_headingPIDOutput = m_drive.getRotationController().calculate(m_drive.getAngleHeading(), Robot.shuffleboard.getRequestedHeading()); // should be in rad/s
    
    // headingOutput is in rad/s. Need to convert to m/s by multiplying by radius
    m_drive.m_headingPIDOutput *= Math.sqrt(0.5) * Constants.drive.kTrackWidth;

    m_drive.m_swerveModuleStates = new SwerveModuleState[] {
      new SwerveModuleState(m_drive.m_headingPIDOutput, new Rotation2d(Units.degreesToRadians(135))),
      new SwerveModuleState(m_drive.m_headingPIDOutput, new Rotation2d(Units.degreesToRadians(45))),
      new SwerveModuleState(m_drive.m_headingPIDOutput, new Rotation2d(Units.degreesToRadians(225))),
      new SwerveModuleState(m_drive.m_headingPIDOutput, new Rotation2d(Units.degreesToRadians(315)))
    };

    m_drive.setModuleStates(m_drive.m_swerveModuleStates);
  }

  private void testDriveVel() {

    double value = Robot.shuffleboard.getRequestedVelocity();
    for (int i = 0; i < 4; i++) {
      Robot.drive.m_modules[i].setDriveVelocity(value);
    }
    Robot.drive.m_modules[0].setSteerAngle(new Rotation2d(Units.degreesToRadians(135)));
    Robot.drive.m_modules[1].setSteerAngle(new Rotation2d(Units.degreesToRadians(45)));
    Robot.drive.m_modules[2].setSteerAngle(new Rotation2d(Units.degreesToRadians(225)));
    Robot.drive.m_modules[3].setSteerAngle(new Rotation2d(Units.degreesToRadians(315)));
  }

  private void testDriveVolts() {

    double value = Robot.shuffleboard.getRequestedVolts();
    for (int i = 0; i < 4; i++) {
      Robot.drive.m_modules[i].setDriveVoltage(value);
    }
    Robot.drive.m_modules[0].setSteerAngle(new Rotation2d(Units.degreesToRadians(135)));
    Robot.drive.m_modules[1].setSteerAngle(new Rotation2d(Units.degreesToRadians(45)));
    Robot.drive.m_modules[2].setSteerAngle(new Rotation2d(Units.degreesToRadians(225)));
    Robot.drive.m_modules[3].setSteerAngle(new Rotation2d(Units.degreesToRadians(315)));
  }

  private void testSteerAngle() {
    double value = Robot.shuffleboard.getRequestedSteerAngle();
    for (int i = 0; i < 4; i++) {
      Robot.drive.m_modules[i].setDriveVoltage(0);
      Robot.drive.m_modules[i].setSteerAngle(new Rotation2d(Units.degreesToRadians(value)));
    }
  }

  private void testSteerVolts() {
    double value = Robot.shuffleboard.getRequestedVolts();
    for (int i = 0; i < 4; i++) {
      Robot.drive.m_modules[i].setDriveVoltage(0);
      Robot.drive.m_modules[i].setSteerVoltage(value);
    }
  }

}
