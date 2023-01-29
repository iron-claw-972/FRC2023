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
  
  private double m_translationalSenseitivity = Constants.oi.kTranslationalSenseitivity;
  private double m_translationalExpo = Constants.oi.kTranslationalExpo;
  private double m_translationalDeadband = Constants.oi.kTranslationalDeadband;
  private double m_translationalSlewrate = Constants.oi.kTranslationalSlewrate;

  private double m_rotationSenseitiviy = Constants.oi.kRotationSenseitiviy;
  private double m_rotationExpo = Constants.oi.kRotationExpo;
  private double m_rotationDeadband = Constants.oi.kRotationDeadband;
  private double m_rotationSlewrate = Constants.oi.kRotationSlewrate;

  private double m_headingSenseitiviy = Constants.oi.kHeadingSenseitiviy;
  private double m_headingExpo = Constants.oi.kHeadingExpo;
  private double m_headingDeadband = Constants.oi.kHeadingDeadband;
  private double m_previousHeading = 0;

  private DynamicSlewRateLimiter m_xspeedLimiter = new DynamicSlewRateLimiter(m_translationalSlewrate);
  private DynamicSlewRateLimiter m_yspeedLimiter = new DynamicSlewRateLimiter(m_translationalSlewrate);
  private DynamicSlewRateLimiter m_rotLimiter = new DynamicSlewRateLimiter(m_rotationSlewrate);
  private DynamicSlewRateLimiter m_headingLimiter = new DynamicSlewRateLimiter(m_headingSenseitiviy);

  public void configureControls() {
    driver.get(Button.START).onTrue(new InstantCommand(() -> Robot.drive.setPigeonYaw(Constants.drive.kStartingHeadingDegrees)));
  }

  public double getForwardTranslation() {
    return m_yspeedLimiter.calculate(-Functions.expoMS(Functions.deadband(getRawForwardTranslation(), m_translationalDeadband), m_translationalExpo) * Constants.drive.kMaxSpeed * m_translationalSenseitivity, m_translationalSlewrate);
  }

  public double getSideTranslation() {
    return m_xspeedLimiter.calculate(-Functions.expoMS(Functions.deadband(getRawSideTranslation(), m_translationalDeadband), m_translationalExpo) * Constants.drive.kMaxSpeed * m_translationalSenseitivity, m_translationalSlewrate);
  }

  public double getRotation() {
    return m_rotLimiter.calculate(-Functions.expoMS(Functions.deadband(getRawRightX(), m_rotationDeadband), m_rotationExpo) * Constants.drive.kMaxAngularSpeed * m_rotationSenseitiviy, m_rotationSlewrate);
  }

  public double getHeading(){
    if (Functions.calculateHypotenuse(getRawSideTranslation(), getRawForwardTranslation()) <= m_headingDeadband) return m_previousHeading;
    m_headingLimiter.enableContinuous(true);
    m_headingLimiter.setContinuousLimits(-Math.PI,Math.PI);
    m_headingLimiter.setRateLimit(m_headingSenseitiviy *Functions.calculateHypotenuse(getRawSideTranslation(), getRawForwardTranslation()));
    m_previousHeading = m_headingLimiter.calculate(Functions.calculateAngle(getRawSideTranslation(), getRawForwardTranslation()),Functions.expoMS(Functions.calculateHypotenuse(getRawSideTranslation(), getRawForwardTranslation()), m_headingExpo));
    return m_previousHeading;
  }
  
/*
  public double getForwardTranslationSlew() {
    return -m_xspeedLimiter.calculate(Functions.deadband(getRawLeftY(), Constants.oi.kDeadband)) * Constants.drive.kMaxSpeed * 0.1;
  }

  public double getSideTranslationSlew() {
    return -m_yspeedLimiter.calculate(Functions.deadband(getRawLeftX(), Constants.oi.kDeadband)) * Constants.drive.kMaxSpeed * 0.1;
  }

  public double getRotationSlew() {
    return -m_rotLimiter.calculate(Functions.deadband(getRawRightX(), Constants.oi.kDeadband)) * Constants.drive.kMaxAngularSpeed * 0.1;
  }
*/


  public double getRawSideTranslation() {
    return driver.get(Axis.LEFT_X);
  }
  public double getRawForwardTranslation() {
    return driver.get(Axis.LEFT_Y);
  }
  
  public double getRawRightX() {
    return driver.get(Axis.RIGHT_X);
  }
  public double getRawRightY() {
    return driver.get(Axis.RIGHT_Y);
  }
  
  public void updateSettings(){
    m_translationalSenseitivity = Robot.shuffleboard.getTranslationalSenseitivity();
    m_translationalExpo = Robot.shuffleboard.getTranslationalExpo();
    m_translationalDeadband = Robot.shuffleboard.getTranslationalDeadband();
    m_translationalSlewrate = Robot.shuffleboard.getTranslationalSlewrate();

    m_rotationSenseitiviy = Robot.shuffleboard.getRotationSenseitiviy();
    m_rotationExpo = Robot.shuffleboard.getRotationExpo();
    m_rotationDeadband = Robot.shuffleboard.getRotationDeadband();
    m_rotationSlewrate = Robot.shuffleboard.getRotationSlewrate();

    m_headingSenseitiviy = Robot.shuffleboard.getHeadingSenseitiviy();
    m_headingExpo = Robot.shuffleboard.getHeadingExpo();
    m_headingDeadband = Robot.shuffleboard.getHeadingDeadband();
  }

}
