// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.Constants;
import frc.robot.util.LogManager;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private Command m_autoCommand;
  private RobotContainer m_robotContainer;

  public enum RobotId {
    Default, SwerveCompetition, SwerveTest,
    ClassBot1, ClassBot2, ClassBot3, ClassBot4
  };
  public static RobotId kRobotId = RobotId.Default;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Determine the Robot Identity from Preferences
    // To Set the Robot Name
    //   SimGUI: Persistent Values, Preferences, RobotId, then restart Simulation
    //     changes networktables.json, networktables.json.bck (both Untracked)
    // set the default preference to something safe
    Preferences.setString(Constants.kRobotIdKey, RobotId.SwerveCompetition.toString());

    if (!Preferences.containsKey(Constants.kRobotIdKey)) {
      Preferences.setString(Constants.kRobotIdKey, RobotId.Default.toString());
    }
    // get the RobotId from Preferences
    String strId = Preferences.getString(Constants.kRobotIdKey, RobotId.Default.toString());
    // match the string to an RobotId
    for (RobotId rid : RobotId.values()) {
      // does it match the preference string?
      if (strId.equals(rid.name())) {
        // yes, so it is the RobotId
        kRobotId = rid;
      }
    }

    // TODO: Remove this line when someone is able to test whether or not preferences are working
    //kRobotId = RobotId.SwerveTest;
    // report the RobotId to the SmartDashboard
    SmartDashboard.putString("RobotID", kRobotId.name());

    // build the RobotContainer
    m_robotContainer = new RobotContainer();
  }
 
  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    LogManager.log();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically when the robot is disabled */
  @Override
  public void disabledPeriodic() {}

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {

    // When auto starts, we know the start position, so we should always reset the yaw to face the right way.
    m_robotContainer.initDriveYaw(true);

    // Get the autonomous command.
    // This access is fast (about 14 microseconds) because the value is already resident in the Network Tables.
    // There was a problem last year because the operation also installed about over a dozen items (taking more than 20 ms).
    m_autoCommand = m_robotContainer.getAutonomousCommand();

    // If there is an autonomous command, then schedule it
    if (m_autoCommand != null) {
      m_autoCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }
  

  /** This function is called once each time the robot enters Teleop mode. */
  @Override
  public void teleopInit() {

    // If we enable teleop for the first time, we should reset the drive yaw. Mainly for testing.
    //m_robotContainer.initDriveYaw(false);

    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autoCommand != null) {
      m_autoCommand.cancel();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
  }

  /** This function is called once each time the robot enters Test mode. */
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
