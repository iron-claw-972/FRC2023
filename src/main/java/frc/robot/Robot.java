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
import frc.robot.controls.Driver;
import frc.robot.controls.Operator;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.LogManager;
import frc.robot.util.Node;
import frc.robot.util.PathGroupLoader;
import frc.robot.util.ShuffleboardManager;
import frc.robot.util.Vision;
import lib.controllers.GameController.DPad;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autoCommand;
  public static ShuffleboardManager shuffleboard;
  public static Drivetrain drive;

  private static boolean isTestMode = false;
  // Array of april tags. The index of the april tag in the array is equal to its id, and aprilTags[0] is null.
  public final static Pose3d[] aprilTags = new Pose3d[9];

  // 2D arrays of nodes. blueNodes[3][1] will return the top row cone node on the far left side (from the perspective of the driver)
  public final static Node[][] blueNodes = new Node[4][];
  public final static Node[][] redNodes = new Node[4][];

  // Where the robot will score.
  public static Node selectedNode = null;

  /// Selection values (grid, row, spot)
  public static int[] selectValues = {0,0,0};

  // Timer for clearing array
  public static double selectTime;

  // How much time it should take (in frames)
  public final static double selectTimeAmount=100;

  // Possible teams
  public static enum Teams {BLUE, RED};
  public static Teams team;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    // This is really annoying so it's disabled
    DriverStation.silenceJoystickConnectionWarning(true);

    // load paths before auto starts
    PathGroupLoader.loadPathGroups();

    // make subsystems
    shuffleboard = new ShuffleboardManager();
    drive = new Drivetrain(new AHRS());

    shuffleboard.setup();
    Vision.setup();

    // Puts April tags in array
    for(int i = 1; i <= 8; i++){
      aprilTags[i]=Vision.getTagPose(i);
    }

    // Puts nodes in arrays
    for(int i = 1; i <= 3; i++){
      blueNodes[i] = new Node[10];
      redNodes[i] = new Node[10];
      for(int j = 1; j <= 9; j++){
        blueNodes[i][j] = new Node(Teams.BLUE, i, j);
        redNodes[i][j] = new Node(Teams.RED, i, j);
      }
    }

    // Sets robot pose to 1 meter in front of april tag 2
    drive.resetPose(aprilTags[2].getX()-1, aprilTags[2].getY(), 0);

    Driver.configureControls();
    Operator.configureControls();
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
    selectTime--;
    if(selectTime==0){
      selectValues[0]=0;
      selectValues[1]=0;
      selectValues[2]=0;
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
  public static void DPadPress(DPad direction){
    if(direction==DPad.DOWN){
      selectTime=1;
    }else{
      selectTime=selectTimeAmount;
      int pressValue = direction==DPad.LEFT?1:direction==DPad.UP?2:3;
      if(selectValues[0]==0){
        selectValues[0]=pressValue;
      }else if(selectValues[1]==0){
        selectValues[1]=pressValue;
      }else{
        selectValues[2]=pressValue;
        selectTime=1;
        if(team==Teams.BLUE){
          selectedNode=blueNodes[selectValues[1]][selectValues[0]*3-3+selectValues[2]];
        }else{
          selectedNode=redNodes[selectValues[1]][selectValues[0]*3-3+selectValues[2]];
        }
      }
    }
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    CommandScheduler.getInstance().cancelAll();
    isTestMode = false;
  }

  @Override
  public void disabledPeriodic() {
    m_autoCommand = getAutonomousCommand();
    team = getTeam();
  }

  /** This autonomous runs the autonomous command selected by your {@link Robot} class. */
  @Override
  public void autonomousInit() {
    isTestMode = false;
    if (m_autoCommand != null) {
      m_autoCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autoCommand != null) {
      m_autoCommand.cancel();
    }
    isTestMode = false;
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();

    // it may be needed to disable LiveWindow (we don't use it anyway)
    //LiveWindow.setEnabled(false)

    isTestMode = true;

  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return shuffleboard.getAutonomousCommand();
  }
  public Teams getTeam() {
    return shuffleboard.getTeam();
  }

  public static boolean isTestMode() {
    return isTestMode;
  }
}
