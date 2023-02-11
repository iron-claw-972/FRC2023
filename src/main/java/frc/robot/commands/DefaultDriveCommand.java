package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.constants.DriveConstants;
import frc.robot.controls.Driver;
import frc.robot.subsystems.Drivetrain;

public class DefaultDriveCommand extends CommandBase {
  private final Drivetrain m_drive;

  public DefaultDriveCommand(Drivetrain drive) {
    m_drive = drive;

    addRequirements(drive);
  }

  @Override
  public void execute() {

    // TODO: rewrite these as commands and add them to the test tab in robot container
    // if (Robot.shuffleboard.getTestModeType() == TestType.HEADING_PID) {
    //   runHeadingPID();
    //   return;
    // } else if (Robot.shuffleboard.getTestModeType() == TestType.MODULE_DRIVE_VELOCITY) {
    //   testDriveVel();
    //   Robot.shuffleboard.setDriveModuleFeedforward();
    //   return;
    // } else if (Robot.shuffleboard.getTestModeType() == TestType.MODULE_STEER_ANGLE){
    //   testSteerAngle();
    //   Robot.shuffleboard.setSteerModuleFeedforward();
    //   return;
    // } else if (Robot.shuffleboard.getTestModeType() == TestType.DRIVE_VOLTAGE){
    //   testDriveVolts();
    //   return;
    // } else if (Robot.shuffleboard.getTestModeType() == TestType.STEER_VOLTAGE){
    //   testSteerVolts();
    //   return;
    // }
    // if (Robot.shuffleboard.getTestModeType() == TestType.HEADING_DRIVE){
    //   rot = Driver.getHeading();
    //   m_drive.driveHeading(xSpeed, ySpeed, rot, fieldRelative);
    //   return;
    // }

    m_drive.setAllOptimize(true);
    double xSpeed = Driver.getForwardTranslation();
    double ySpeed = Driver.getSideTranslation();
    double rot = Driver.getRotation();
    boolean fieldRelative = Driver.getFieldRelative();

    // System.out.println("driving: " + xSpeed + "," + ySpeed + "," + rot +
    // "," + Driver.getRawForwardTranslation() + "," + Driver.getRawSideTranslation() +","+ Driver.getRawRotation());
    m_drive.driveRot(xSpeed, ySpeed, rot, fieldRelative);
}

  @Override
  public void end(boolean interrupted) {
    m_drive.driveRot(0.0, 0.0, 0.0, false);
  }

  // TODO: ALL OF THE FUNCTIONS BELOW SHOULD BE IN DRIVETRAIN.JAVA AND PART OF SOME OTHER TEST COMMAND

  private void runHeadingPID() {
    
  }

  private void testDriveVel() {
    m_drive.setAllOptimize(true);
    double value = 0; //TODO: fix, was: Robot.shuffleboard.getRequestedDriveVelocity();
    for (int i = 0; i < 4; i++) {
      m_drive.m_modules[i].setDriveVelocity(value);
    }
    m_drive.m_modules[0].setSteerAngle(new Rotation2d(Units.degreesToRadians(135)));
    m_drive.m_modules[1].setSteerAngle(new Rotation2d(Units.degreesToRadians(45)));
    m_drive.m_modules[2].setSteerAngle(new Rotation2d(Units.degreesToRadians(225)));
    m_drive.m_modules[3].setSteerAngle(new Rotation2d(Units.degreesToRadians(315)));
  }

  private void testSteerAngle() {
    m_drive.setAllOptimize(true);
    double value = 0; //TODO: fix, was: Robot.shuffleboard.getRequestedSteerAngle();
    for (int i = 0; i < 4; i++) {
      m_drive.m_modules[i].setDriveVoltage(0);
      m_drive.m_modules[i].setSteerAngle(new Rotation2d(value));
    }
  }

  private void testSteerVolts() {
    m_drive.setAllOptimize(true);
    double value = 0; //TODO fix, was: Robot.shuffleboard.getRequestedVolts();
    for (int i = 0; i < 4; i++) {
      m_drive.m_modules[i].setDriveVoltage(0);
      m_drive.m_modules[i].setSteerVoltage(value);
    }
  }

  private void testFFandPID(){
    if(false) //TODO: do better than this
    // Robot.shuffleboard.getRequestedDriveVelocity()+0.1>m_drive.m_modules[0].getDriveVelocity()||
    // Robot.shuffleboard.getRequestedDriveVelocity()-0.1<m_drive.m_modules[0].getDriveVelocity()||
    // Robot.shuffleboard.getRequestedDriveVelocity()+0.1>m_drive.m_modules[1].getDriveVelocity()||
    // Robot.shuffleboard.getRequestedDriveVelocity()-0.1<m_drive.m_modules[1].getDriveVelocity()||
    // Robot.shuffleboard.getRequestedDriveVelocity()+0.1>m_drive.m_modules[2].getDriveVelocity()||
    // Robot.shuffleboard.getRequestedDriveVelocity()-0.1<m_drive.m_modules[2].getDriveVelocity()||
    // Robot.shuffleboard.getRequestedDriveVelocity()+0.1>m_drive.m_modules[3].getDriveVelocity()||
    // Robot.shuffleboard.getRequestedDriveVelocity()-0.1<m_drive.m_modules[3].getDriveVelocity()
    {
      System.out.println("ERROR VELOCITY INCORRECT");
    }
  }
}
