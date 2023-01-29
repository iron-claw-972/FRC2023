package frc.robot.controls;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.commands.SetFormationX;
import frc.robot.constants.Constants;
import frc.robot.util.DynamicSlewRateLimiter;
import frc.robot.util.Functions;
import lib.controllers.Ex3DProController;
import lib.controllers.GameController;
import lib.controllers.MadCatzController;
import lib.controllers.Ex3DProController.Ex3DProAxis;
import lib.controllers.Ex3DProController.Ex3DProButton;
import lib.controllers.GameController.GCAxis;
import lib.controllers.GameController.GCButton;
import lib.controllers.MadCatzController.MadCatzAxis;
import lib.controllers.MadCatzController.MadCatzButton;

public class Driver {
  private static GameController driverGC = new GameController(Constants.oi.kDriverJoy);
  private static Ex3DProController driverEPC = new Ex3DProController(Constants.oi.kDriverJoy);
  private static MadCatzController driverMCC = new MadCatzController(Constants.oi.kDriverJoy);
  
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

  public Driver(){
    m_headingLimiter.enableContinuous(true);
    m_headingLimiter.setContinuousLimits(-Math.PI,Math.PI);
  }

  public void configureControls() {
    driverGC.get(GCButton.START).onTrue(new InstantCommand(() -> Robot.drive.setPigeonYaw(Constants.drive.kStartingHeadingDegrees)));
    driverGC.get(GCButton.A).whileTrue(new SetFormationX(Robot.drive));
    driverEPC.get(Ex3DProButton.B1).whileTrue(new SetFormationX(Robot.drive));
    driverMCC.get(MadCatzButton.B1).whileTrue(new SetFormationX(Robot.drive));
  }

  public double getForwardTranslation() {
    return m_yspeedLimiter.calculate(-Functions.expoMS(Functions.deadband(getRawForwardTranslation(), m_translationalDeadband), m_translationalExpo) * Constants.drive.kMaxSpeed * m_translationalSenseitivity, m_translationalSlewrate);
  }
  public double getSideTranslation() {
    return m_xspeedLimiter.calculate(-Functions.expoMS(Functions.deadband(getRawSideTranslation(), m_translationalDeadband), m_translationalExpo) * Constants.drive.kMaxSpeed * m_translationalSenseitivity, m_translationalSlewrate);
  }
  public double getRotation() {
    return m_rotLimiter.calculate(-Functions.expoMS(Functions.deadband(getRawRotation(), m_rotationDeadband), m_rotationExpo) * Constants.drive.kMaxAngularSpeed * m_rotationSenseitiviy, m_rotationSlewrate);
  }
  public double getHeading(){
    if (Functions.calculateHypotenuse(getRawSideTranslation(), getRawForwardTranslation()) <= m_headingDeadband) return m_previousHeading;
    m_headingLimiter.setRateLimit(m_headingSenseitiviy *getRawHeadingMagnitude());
    m_previousHeading = m_headingLimiter.calculate(getRawHeadingAngle(),Functions.expoMS(getRawHeadingMagnitude(), m_headingExpo));
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
    if (Robot.shuffleboard.getControllerType() instanceof GameController) return driverGC.get(GCAxis.LEFT_X);
    if (Robot.shuffleboard.getControllerType() instanceof Ex3DProController) return driverEPC.get(Ex3DProAxis.X);
    if (Robot.shuffleboard.getControllerType() instanceof MadCatzController) return driverMCC.get(MadCatzAxis.X);
    return 0;
  }
  public double getRawForwardTranslation() {
    if (Robot.shuffleboard.getControllerType() instanceof GameController) return driverGC.get(GCAxis.LEFT_Y);
    if (Robot.shuffleboard.getControllerType() instanceof Ex3DProController) return driverEPC.get(Ex3DProAxis.Y);
    if (Robot.shuffleboard.getControllerType() instanceof MadCatzController) return driverMCC.get(MadCatzAxis.Y);
    return 0;

  }
  public double getRawRotation(){
    if (Robot.shuffleboard.getControllerType() instanceof GameController) return driverGC.get(GCAxis.RIGHT_X);
    if (Robot.shuffleboard.getControllerType() instanceof Ex3DProController) return driverEPC.get(Ex3DProAxis.Z);
    if (Robot.shuffleboard.getControllerType() instanceof MadCatzController) return driverMCC.get(MadCatzAxis.ZROTATE);
    return 0;
  }
  public double getRawHeadingAngle() {
    if (Robot.shuffleboard.getControllerType() instanceof GameController) return Functions.calculateAngle(driverGC.get(GCAxis.RIGHT_X),driverGC.get(GCAxis.RIGHT_Y));
    return 0;
  }
  public double getRawHeadingMagnitude() {
    if (Robot.shuffleboard.getControllerType() instanceof GameController) return Functions.calculateHypotenuse(driverGC.get(GCAxis.RIGHT_X),driverGC.get(GCAxis.RIGHT_Y));
    return 0;
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
