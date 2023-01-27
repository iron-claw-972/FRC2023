package frc.robot.controls;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.util.Functions;
import lib.controllers.GameController;
import lib.controllers.GameController.Axis;
import lib.controllers.GameController.Button;

public class Driver {
  private static GameController driver = new GameController(Constants.oi.kDriverJoy);
  
  private static final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(0.1);
  private static final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(0.1);
  private static final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(0.1);

  public static void configureControls() {
    driver.get(Button.START).onTrue(new InstantCommand(() -> Robot.drive.setPigeonYaw(Constants.drive.kStartingHeadingDegrees)));
  }

  public static double getForwardTranslation() {
    return -Functions.expoMS(2, Functions.deadband(getRawLeftY(), Constants.oi.kDeadband)) * Constants.drive.kMaxSpeed * 0.1;
  }

  public static double getSideTranslation() {
    return -Functions.expoMS(2, Functions.deadband(getRawLeftX(), Constants.oi.kDeadband)) * Constants.drive.kMaxSpeed * 0.1;
  }

  public static double getRotation() {
    return -Functions.expoMS(2, Functions.deadband(getRawRightX(), Constants.oi.kDeadband)) * Constants.drive.kMaxAngularSpeed * 0.1;
  }

  public static double getForwardTranslationSlew() {
    return -m_xspeedLimiter.calculate(Functions.deadband(getRawLeftY(), Constants.oi.kDeadband)) * Constants.drive.kMaxSpeed * 0.1;
  }

  public static double getSideTranslationSlew() {
    return -m_yspeedLimiter.calculate(Functions.deadband(getRawLeftX(), Constants.oi.kDeadband)) * Constants.drive.kMaxSpeed * 0.1;
  }

  public static double getRotationSlew() {
    return -m_rotLimiter.calculate(Functions.deadband(getRawRightX(), Constants.oi.kDeadband)) * Constants.drive.kMaxAngularSpeed * 0.1;
  }

  public static double getRawRightX() {
    return driver.get(Axis.RIGHT_X);
  }

  public static double getRawLeftX() {
    return driver.get(Axis.LEFT_X);
  }

  public static double getRawLeftY() {
    return driver.get(Axis.LEFT_Y);
  }

}
