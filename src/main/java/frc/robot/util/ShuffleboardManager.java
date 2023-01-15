package frc.robot.util;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Robot;
import frc.robot.commands.FeedForwardCharacterization;
import frc.robot.commands.FeedForwardCharacterization.FeedForwardCharacterizationData;
import frc.robot.commands.auto.PathPlannerCommand;

public class ShuffleboardManager {

  SendableChooser<Command> m_autoCommand = new SendableChooser<>();

  ShuffleboardTab m_mainTab = Shuffleboard.getTab("Main");
  public ShuffleboardTab m_driveTab = Shuffleboard.getTab("Drive");
  ShuffleboardTab m_swerveModulesTab = Shuffleboard.getTab("Swerve Modules");
  ShuffleboardTab m_autoTab = Shuffleboard.getTab("Auto");

  GenericEntry m_heading = m_driveTab.add("Set Heading (-pi to pi)", 0).getEntry();
  GenericEntry m_velocity = m_swerveModulesTab.add("Set Drive Velocity", 0).getEntry();
  GenericEntry m_turn = m_swerveModulesTab.add("Set Drive Turn", 0).getEntry();
  GenericEntry m_staticFeedforward = m_swerveModulesTab.add("Set Static Feedforward", 0).getEntry();
  GenericEntry m_velFeedforward = m_swerveModulesTab.add("Set Vel Feedforward", 0).getEntry();

  GenericEntry m_commandScheduler = m_mainTab.add("Command Scheduler", "NULL").getEntry();

  SendableChooser<PracticeModeType> m_practiceMode = new SendableChooser<>();
  
  public void setup() {
    LiveWindow.disableAllTelemetry(); // LiveWindow is causing periodic loop overruns

    autoChooserUpdate();
    practiceChooserUpdate();

    m_autoTab.add("Auto Chooser", m_autoCommand);
    m_mainTab.add("Practice Mode Type Chooser", m_practiceMode);

    setupDrivetrain();

    m_driveTab.add("xController", Robot.drive.getXController());
    m_driveTab.add("yController", Robot.drive.getYController());
    m_driveTab.add("rotationController", Robot.drive.getRotationController());
    m_driveTab.addNumber("getAngle", () -> Robot.drive.getAngleHeading());
    m_driveTab.addNumber("heading PID output", () -> Robot.drive.headingPIDOutput);
  }

  public Command getAutonomousCommand() {
    return m_autoCommand.getSelected();
  }

  public void autoChooserUpdate() {
    m_autoCommand.addOption("Do Nothing", new PrintCommand("This will do nothing!"));
    // m_autoCommand.setDefaultOption("TestAuto", new PathPlannerCommand("TestAuto", 0)); 
    m_autoCommand.addOption("FeedForwardCharacterization", new FeedForwardCharacterization(Robot.drive, true, new FeedForwardCharacterizationData("drive"), Robot.drive::runCharacterizationVolts, Robot.drive::getCharacterizationVelocity));
  }

  public PracticeModeType getPracticeModeType() {
    return m_practiceMode.getSelected();
  }

  public void practiceChooserUpdate() {
    m_practiceMode.addOption(PracticeModeType.TUNE_HEADING_PID.toString(), PracticeModeType.TUNE_HEADING_PID);
    m_practiceMode.addOption(PracticeModeType.TUNE_MODULE_DRIVE.toString(), PracticeModeType.TUNE_MODULE_DRIVE);
    m_practiceMode.addOption(PracticeModeType.TUNE_MODULE_TURN.toString(), PracticeModeType.TUNE_MODULE_TURN);
    m_practiceMode.setDefaultOption(PracticeModeType.NONE.toString(), PracticeModeType.NONE);
  }

  public double getRequestedHeading() {
    return m_heading.getDouble(0);
  }

  public double getRequestedVelocity() {
    return m_velocity.getDouble(0);
  }

  public double getRequestedTurnAngle() {
    return m_turn.getDouble(0);
  }
  public double getStaticFeedforward() {
    return m_staticFeedforward.getDouble(0);
  }
  public double getVelocityFeedforward() {
    return m_velFeedforward.getDouble(0);
  }


  public void loadCommandSchedulerShuffleboard(){
    CommandScheduler.getInstance().onCommandInitialize(command -> m_commandScheduler.setString(command.getName() + " initialized."));
    CommandScheduler.getInstance().onCommandInterrupt(command -> m_commandScheduler.setString(command.getName() + " interrupted."));
    CommandScheduler.getInstance().onCommandFinish(command -> m_commandScheduler.setString(command.getName() + " finished."));
  }

  private void setupDrivetrain() {
    m_swerveModulesTab.addNumber("Angle Front Left",  () -> Units.radiansToDegrees(Robot.drive.m_modules[0].getAngle()));
    m_swerveModulesTab.addNumber("Angle Front Right", () -> Units.radiansToDegrees(Robot.drive.m_modules[1].getAngle()));
    m_swerveModulesTab.addNumber("Angle Back Left",   () -> Units.radiansToDegrees(Robot.drive.m_modules[2].getAngle()));
    m_swerveModulesTab.addNumber("Angle Back Right",  () -> Units.radiansToDegrees(Robot.drive.m_modules[3].getAngle()));

    m_driveTab.addNumber("Gyro X", () -> Robot.drive.getAngularRate(0));
    m_driveTab.addNumber("Gyro Y", () -> Robot.drive.getAngularRate(1));
    m_driveTab.addNumber("Gyro Z", () -> Robot.drive.getAngularRate(2));

    // m_swerveModulesTab.addNumber("FL desired speed", () -> Robot.drive.swerveModuleStates[0].speedMetersPerSecond);
    // m_swerveModulesTab.addNumber("FR desired speed", () -> Robot.drive.swerveModuleStates[1].speedMetersPerSecond);
    // m_swerveModulesTab.addNumber("BL desired speed", () -> Robot.drive.swerveModuleStates[2].speedMetersPerSecond);
    // m_swerveModulesTab.addNumber("BR desired speed", () -> Robot.drive.swerveModuleStates[3].speedMetersPerSecond);

    // m_swerveModulesTab.addNumber("FL desired angle", () -> Robot.drive.swerveModuleStates[0].angle.getDegrees());
    // m_swerveModulesTab.addNumber("FR desired angle", () -> Robot.drive.swerveModuleStates[1].angle.getDegrees());
    // m_swerveModulesTab.addNumber("BL desired angle", () -> Robot.drive.swerveModuleStates[2].angle.getDegrees());
    // m_swerveModulesTab.addNumber("BR desired angle", () -> Robot.drive.swerveModuleStates[3].angle.getDegrees());

    // m_swerveModulesTab.addNumber("FL FF", () -> Robot.drive.m_modules[0].getTurnFeedForward());
    // m_swerveModulesTab.addNumber("FR FF", () -> Robot.drive.m_modules[1].getTurnFeedForward());
    // m_swerveModulesTab.addNumber("BL FF", () -> Robot.drive.m_modules[2].getTurnFeedForward());
    // m_swerveModulesTab.addNumber("BR FF", () -> Robot.drive.m_modules[3].getTurnFeedForward());

    m_swerveModulesTab.addNumber("FL PID Output", () -> Robot.drive.m_modules[0].getTurnOutput());
    m_swerveModulesTab.addNumber("FR PID Output", () -> Robot.drive.m_modules[1].getTurnOutput());
    m_swerveModulesTab.addNumber("BL PID Output", () -> Robot.drive.m_modules[2].getTurnOutput());
    m_swerveModulesTab.addNumber("BR PID Output", () -> Robot.drive.m_modules[3].getTurnOutput());


    m_swerveModulesTab.addNumber("Vel Front Right", () -> Robot.drive.m_modules[0].getDriveVelocity());
    m_swerveModulesTab.addNumber("Vel Front Left", () -> Robot.drive.m_modules[1].getDriveVelocity());
    m_swerveModulesTab.addNumber("Vel Back Right", () -> Robot.drive.m_modules[2].getDriveVelocity());
    m_swerveModulesTab.addNumber("Vel Back Left", () -> Robot.drive.m_modules[3].getDriveVelocity());
  }

}
