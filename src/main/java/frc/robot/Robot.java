// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer.Teams;
import frc.robot.util.LogManager;
import frc.robot.util.Node;
import frc.robot.util.PathGroupLoader;
import frc.robot.util.Vision;
import lib.controllers.GameController.DPad;
// import frc.robot.RobotContainer.Teams;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autoCommand;
  
  
  // Where the robot will score.
  

  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    RobotContainer.selectTime--;
    if(RobotContainer.selectTime==0){
      RobotContainer.selectValues[0]=0;
      RobotContainer.selectValues[1]=0;
      RobotContainer.selectValues[2]=0;
    }
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    LogManager.log();
  }

    /**
   * Method to store DPad values and use them to set selectedNode
   * Down clears the array
   * Left is 1, up is 2, and right is 3 for selection
   * For example, up right left will select the center grid, top row, and left spot.
   * @param direction = Which DPad button is pressed
   */
  public static void DPadPress(DPad direction) { 
    if (direction==DPad.DOWN) {
      RobotContainer.selectTime=1;
    } else {
      RobotContainer.selectTime = RobotContainer.selectTimeAmount;
      int pressValue = direction == DPad.LEFT?1:direction==DPad.UP?2:3;

      if (RobotContainer.selectValues[0]==0){
        RobotContainer.selectValues[0]=pressValue;
      } else if (RobotContainer.selectValues[1]==0) {
        RobotContainer.selectValues[1]=pressValue;
      } else {
        RobotContainer.selectValues[2] = pressValue;
        RobotContainer.selectTime = 1;

        if (RobotContainer.team == RobotContainer.Teams.BLUE) {
          RobotContainer.selectedNode = RobotContainer.blueNodes[RobotContainer.selectValues[1]][RobotContainer.selectValues[0]*3-3+RobotContainer.selectValues[2]];
        }else{
          RobotContainer.selectedNode = RobotContainer.redNodes[RobotContainer.selectValues[1]][RobotContainer.selectValues[0]*3-3+RobotContainer.selectValues[2]];
        }
      }
    }
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically when the robot is disabled */
  @Override
  public void disabledPeriodic() {
    RobotContainer.team = m_robotContainer.getTeam();
    m_autoCommand = m_robotContainer.getAutonomousCommand(); // update the auto command before auto starts
  }

  /** This autonomous runs the autonomous command selected by your {@link Robot} class. */
  @Override
  public void autonomousInit() {
    if (m_autoCommand != null) {
      m_autoCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once each time the robot enters Teleop mode. */
  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autoCommand != null) {
      m_autoCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once each time the robot enters Test mode. */
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

}
