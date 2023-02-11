// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
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

  private final SingleJointedArmSim armSim = 
  new SingleJointedArmSim(
    Constants.arm.armSimMotor, 
    Constants.arm.armReduction, 
    Constants.arm.armMOI, 
    Constants.arm.armLength, 
    Units.degreesToRadians(0), 
    Units.degreesToRadians(180), 
    Constants.arm.armMass, 
    true
    );
  private double armPositionDeg = 0;
  private double kArmEncoderDistPerPulse = 2.0*Math.PI/8192;
  private final Encoder dummyEncoder = new Encoder(0, 1);
  private final EncoderSim encoderSim = new EncoderSim(dummyEncoder);

  public static final String kArmPositionKey = "ArmPosition";
  public static final String kArmPKey = "ArmP";

  // constructor arguments TBD
  private final Mechanism2d mech2d = new Mechanism2d(60, 60);
  private final MechanismRoot2d armPivot = mech2d.getRoot("ArmPivot", 30, 30);
  private final MechanismLigament2d armTower =
    armPivot.append(new MechanismLigament2d("ArmTower", 30, -90));
  private final MechanismLigament2d armDiagram =
      armPivot.append(
          new MechanismLigament2d(
              "Arm",
              30,
              Units.radiansToDegrees(armSim.getAngleRads()),
              6,
              new Color8Bit(Color.kYellow)));

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
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    LogManager.log();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically when the robot is disabled */
  @Override
  public void disabledPeriodic() {
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

  @Override
  public void simulationInit() {
    
  } 

  @Override
  public void simulationPeriodic() {
    armSim.setInput(Robot.arm.getMotorValue()*RobotController.getBatteryVoltage());
    armSim.update(0.020);
    encoderSim.setDistance(armSim.getAngleRads());
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(armSim.getCurrentDrawAmps()));
    armDiagram.setAngle(Units.radiansToDegrees(armSim.getAngleRads()));
  }
}
