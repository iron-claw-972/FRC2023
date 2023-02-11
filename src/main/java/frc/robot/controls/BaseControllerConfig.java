package frc.robot.controls;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.OIConstants;
import frc.robot.util.DynamicSlewRateLimiter;
import frc.robot.util.Functions;
import lib.controllers.Controller;
import lib.controllers.GameController;

public abstract class BaseControllerConfig {

  public abstract void configureControls();
  public abstract double getRawSideTranslation();
  public abstract double getRawForwardTranslation();
  public abstract double getRawRotation();
  public abstract double getRawHeadingAngle();
  public abstract double getRawHeadingMagnitude();

  private static Controller m_controllerType = new GameController(-1);

  private static DynamicSlewRateLimiter m_xspeedLimiter = new DynamicSlewRateLimiter(OIConstants.kTranslationalSlewrate);
  private static DynamicSlewRateLimiter m_yspeedLimiter = new DynamicSlewRateLimiter(OIConstants.kTranslationalSlewrate);
  private static DynamicSlewRateLimiter m_rotLimiter = new DynamicSlewRateLimiter(OIConstants.kRotationSlewrate);
  private static DynamicSlewRateLimiter m_headingLimiter = new DynamicSlewRateLimiter(OIConstants.kHeadingSensitivity);

  private static GenericEntry m_translationalSensitivityEntry, m_translationalExpoEntry, m_translationalDeadbandEntry, m_translationalSlewrateEntry, m_fieldRelativeEntry;
  private static GenericEntry m_rotationSensitivityEntry, m_rotationExpoEntry, m_rotationDeadbandEntry, m_rotationSlewrateEntry;
  private static GenericEntry m_headingSensitivityEntry, m_headingExpoEntry, m_headingDeadbandEntry, m_consantHeadingMagnitudeEntry;
  private static GenericEntry m_invertEntry;
  private static SendableChooser<Controller> m_controllerTypeChooser = new SendableChooser<>();

  public static double m_previousHeading;

  public BaseControllerConfig()
  {
    m_headingLimiter.setContinuousLimits(-Math.PI,Math.PI);
    m_headingLimiter.enableContinuous(true);
    ShuffleboardTab controllerTab = Shuffleboard.getTab("Controller");

    controllerTab.add("Controller Type", m_controllerType);

    m_translationalSensitivityEntry = controllerTab.add("translationalSensitivity", OIConstants.kTranslationalSenseitivity).getEntry();
    m_translationalExpoEntry = controllerTab.add("translationalExpo", OIConstants.kTranslationalExpo).getEntry();
    m_translationalDeadbandEntry = controllerTab.add("translationalDeadband", OIConstants.kTranslationalDeadband).getEntry();
    m_translationalSlewrateEntry = controllerTab.add("translationalSlewrate", OIConstants.kTranslationalSlewrate).getEntry();
    m_fieldRelativeEntry = controllerTab.add("Field Relative", OIConstants.kFieldRelative).getEntry();

    m_rotationSensitivityEntry = controllerTab.add("rotationSensitiviy", OIConstants.kRotationSenseitiviy).getEntry();
    m_rotationExpoEntry = controllerTab.add("rotationExpo", OIConstants.kRotationExpo).getEntry();
    m_rotationDeadbandEntry = controllerTab.add("rotationDeadband", OIConstants.kRotationDeadband).getEntry();
    m_rotationSlewrateEntry = controllerTab.add("rotationSlewrate", OIConstants.kRotationSlewrate).getEntry();

    m_headingSensitivityEntry = controllerTab.add("headingSensitiviy", OIConstants.kHeadingSensitivity).getEntry();
    m_headingExpoEntry = controllerTab.add("headingExpo", OIConstants.kHeadingExpo).getEntry();
    m_headingDeadbandEntry = controllerTab.add("headingDeadband", OIConstants.kHeadingDeadband).getEntry();
  }

  public double getForwardTranslation() {
    return m_yspeedLimiter.calculate(-Functions.expoMS(Functions.deadband(getRawForwardTranslation(), getTranslationDeadband()), getTranslationExpo()) * DriveConstants.kMaxSpeed * getTranslationSensitivity(), getTranslationSlewrate());
  }
  public double getSideTranslation() {
    return m_xspeedLimiter.calculate(-Functions.expoMS(Functions.deadband(getRawSideTranslation(), getTranslationDeadband()), getTranslationExpo()) * DriveConstants.kMaxSpeed * getTranslationSensitivity(), getTranslationSlewrate());
  }
  public double getRotation() {
    return m_rotLimiter.calculate(-Functions.expoMS(Functions.deadband(getRawRotation(), getRotationDeadband()), getRotationExpo()) * DriveConstants.kMaxAngularSpeed * getRotationSensitiviy(), getRotationSlewrate());
  }

  public double getHeading(){
    if (getRawHeadingMagnitude() <= getHeadingDeadband()) return m_headingLimiter.calculate(getHeading(),1e-6);
    m_previousHeading = m_headingLimiter.calculate(getRawHeadingAngle(), Functions.expoMS(getRawHeadingMagnitude(), getHeadingExpo()) * getHeadingSensitivity());
    return m_previousHeading;
  }

  public static double getTranslationSensitivity(){
    return m_translationalSensitivityEntry.getDouble(OIConstants.kTranslationalSenseitivity);
  }
  public static double getTranslationExpo(){
    return m_translationalExpoEntry.getDouble(OIConstants.kTranslationalExpo);
  }
  public static double getTranslationDeadband(){
    return m_translationalDeadbandEntry.getDouble(OIConstants.kTranslationalDeadband);
  }
  public static double getTranslationSlewrate(){
    return m_translationalSlewrateEntry.getDouble(OIConstants.kTranslationalSlewrate);
  }
  public static boolean getFieldRelative() {
    return m_fieldRelativeEntry.getBoolean(OIConstants.kFieldRelative);
  }
  public static double getRotationSensitiviy(){
    return m_rotationSensitivityEntry.getDouble(OIConstants.kRotationSenseitiviy);
  }
  public static double getRotationExpo(){
    return m_rotationExpoEntry.getDouble(OIConstants.kRotationExpo);
  }
  public static double getRotationDeadband(){
    return m_rotationDeadbandEntry.getDouble(OIConstants.kRotationDeadband);
  }
  public static double getRotationSlewrate(){
    return m_rotationSlewrateEntry.getDouble(OIConstants.kRotationSlewrate);
  }
  public static double getHeadingSensitivity(){
    return m_headingSensitivityEntry.getDouble(OIConstants.kHeadingSensitivity);
  }
  public static double getHeadingExpo(){
    return m_headingExpoEntry.getDouble(OIConstants.kHeadingExpo);
  }
  public static double getHeadingDeadband(){
  return m_headingDeadbandEntry.getDouble(OIConstants.kHeadingDeadband);
  }
  public static boolean getConsantHeadingMagnitude(){
    return m_consantHeadingMagnitudeEntry.getBoolean(OIConstants.kConsantHeadingMagnatuide);
  }
  public static Controller getControllerType(){
    return m_controllerTypeChooser.getSelected();
  }
  public static boolean getInverted(){
    return m_invertEntry.getBoolean(OIConstants.kInvert);
  }
}