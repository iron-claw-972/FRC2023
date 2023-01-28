package frc.robot.controls;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.util.DynamicSlewRateLimiter;
import frc.robot.util.Functions;
import lib.controllers.GameController;
import lib.controllers.GameController.Axis;
import lib.controllers.GameController.Button;

public class Driver {
  private static GameController driver = new GameController(Constants.oi.kDriverJoy);
  
  private static double m_translationalSenseitivity = 0.1;
  private static double m_translationalExpo = 2;
  private static double m_translationalDeadband = 0.05;

  private static double m_rotationSenseitiviy = 0.1;
  private static double m_rotationExpo = 0.1;
  private static double m_rotationDeadband = 0.05;

  private static double m_headingSenseitiviy = Math.PI*4;
  private static double m_headingExpo = 0.1;
  private static double m_headingDeadband = 0.05;
  private static double m_previousHeading = 0;
  
  private static DynamicSlewRateLimiter m_xspeedLimiter = new DynamicSlewRateLimiter(0.1);
  private static DynamicSlewRateLimiter m_yspeedLimiter = new DynamicSlewRateLimiter(0.1);
  private static DynamicSlewRateLimiter m_rotLimiter = new DynamicSlewRateLimiter(0.1);
  private static DynamicSlewRateLimiter m_headingLimiter = new DynamicSlewRateLimiter(m_headingSenseitiviy);

  public static void configureControls() {
    driver.get(Button.START).onTrue(new InstantCommand(() -> Robot.drive.setPigeonYaw(Constants.drive.kStartingHeadingDegrees)));
  }

  public static double getForwardTranslation() {
    return -Functions.expoMS(Functions.deadband(getRawLeftY(), m_translationalDeadband), m_translationalExpo) * Constants.drive.kMaxSpeed * m_translationalSenseitivity;
  }

  public static double getSideTranslation() {
    return -Functions.expoMS(Functions.deadband(getRawLeftX(), m_translationalDeadband), m_translationalExpo) * Constants.drive.kMaxSpeed * m_translationalSenseitivity;
  }

  public static double getRotation() {
    return -Functions.expoMS(Functions.deadband(getRawRightX(), m_rotationDeadband), m_rotationExpo) * Constants.drive.kMaxAngularSpeed * m_rotationSenseitiviy;
  }

  public static double getHeading(){
    if (Functions.calculateHypotenuse(getRawLeftX(), getRawLeftY()) <= m_headingDeadband) return m_previousHeading;
    m_headingLimiter.enableContinuous(true);
    m_headingLimiter.setContinuousLimits(-Math.PI,Math.PI);
    m_headingLimiter.setRateLimit(m_headingSenseitiviy *Functions.calculateHypotenuse(getRawLeftX(), getRawLeftY()));
    m_previousHeading = m_headingLimiter.calculate(Functions.calculateAngle(getRawLeftX(), getRawLeftY()));
    return m_previousHeading;
  }
  
/*
  public static double getForwardTranslationSlew() {
    return -m_xspeedLimiter.calculate(Functions.deadband(getRawLeftY(), Constants.oi.kDeadband)) * Constants.drive.kMaxSpeed * 0.1;
  }

  public static double getSideTranslationSlew() {
    return -m_yspeedLimiter.calculate(Functions.deadband(getRawLeftX(), Constants.oi.kDeadband)) * Constants.drive.kMaxSpeed * 0.1;
  }

  public static double getRotationSlew() {
    return -m_rotLimiter.calculate(Functions.deadband(getRawRightX(), Constants.oi.kDeadband)) * Constants.drive.kMaxAngularSpeed * 0.1;
  }
*/


  
  public static double getRawLeftX() {
    return driver.get(Axis.LEFT_X);
  }
  public static double getRawLeftY() {
    return driver.get(Axis.LEFT_Y);
  }
  public static double getRawRightX() {
    return driver.get(Axis.RIGHT_X);
  }
  public static double getRawRightY() {
    return driver.get(Axis.RIGHT_Y);
  }
  
}
