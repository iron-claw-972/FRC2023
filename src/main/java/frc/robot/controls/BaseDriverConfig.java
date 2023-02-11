package frc.robot.controls;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.OIConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.DynamicSlewRateLimiter;
import frc.robot.util.Functions;

public abstract class BaseDriverConfig {

  private Drivetrain m_drive;

  boolean m_shuffleboardUpdates = false;

  ShuffleboardTab m_controllerTab;
  GenericEntry m_translationalSenseitivityEntry, m_translationalExpoEntry, m_translationalDeadbandEntry, m_translationalSlewrateEntry;
  GenericEntry m_rotationSenseitiviyEntry, m_rotationExpoEntry, m_rotationDeadbandEntry, m_rotationSlewrateEntry;
  GenericEntry m_headingSenseitiviyEntry, m_headingExpoEntry, m_headingDeadbandEntry;

  private double m_translationalSenseitivity = OIConstants.kTranslationalSenseitivity;
  private double m_translationalExpo = OIConstants.kTranslationalExpo;
  private double m_translationalDeadband = OIConstants.kTranslationalDeadband;
  private double m_translationalSlewrate = OIConstants.kTranslationalSlewrate;

  private double m_rotationSenseitiviy = OIConstants.kRotationSenseitiviy;
  private double m_rotationExpo = OIConstants.kRotationExpo;
  private double m_rotationDeadband = OIConstants.kRotationDeadband;
  private double m_rotationSlewrate = OIConstants.kRotationSlewrate;

  private double m_headingSenseitiviy = OIConstants.kHeadingSenseitiviy;
  private double m_headingExpo = OIConstants.kHeadingExpo;
  private double m_headingDeadband = OIConstants.kHeadingDeadband;
  private double m_previousHeading = 0;

  private DynamicSlewRateLimiter m_xspeedLimiter = new DynamicSlewRateLimiter(m_translationalSlewrate);
  private DynamicSlewRateLimiter m_yspeedLimiter = new DynamicSlewRateLimiter(m_translationalSlewrate);
  private DynamicSlewRateLimiter m_rotLimiter = new DynamicSlewRateLimiter(m_rotationSlewrate);
  private DynamicSlewRateLimiter m_headingLimiter = new DynamicSlewRateLimiter(m_headingSenseitiviy);

  public BaseDriverConfig(Drivetrain drive, ShuffleboardTab controllerTab, boolean shuffleboardUpdates){
    m_headingLimiter.setContinuousLimits(-Math.PI,Math.PI);
    m_headingLimiter.enableContinuous(true);
    m_controllerTab = controllerTab;
    m_shuffleboardUpdates = shuffleboardUpdates;
  }

  public double getForwardTranslation() {
    return m_yspeedLimiter.calculate(-Functions.expoMS(Functions.deadband(getRawForwardTranslation(), m_translationalDeadband), m_translationalExpo) * DriveConstants.kMaxSpeed * m_translationalSenseitivity, m_translationalSlewrate);
  }
  public double getSideTranslation() {
    return m_xspeedLimiter.calculate(-Functions.expoMS(Functions.deadband(getRawSideTranslation(), m_translationalDeadband), m_translationalExpo) * DriveConstants.kMaxSpeed * m_translationalSenseitivity, m_translationalSlewrate);
  }
  public double getRotation() {
    return m_rotLimiter.calculate(-Functions.expoMS(Functions.deadband(getRawRotation(), m_rotationDeadband), m_rotationExpo) * DriveConstants.kMaxAngularSpeed * m_rotationSenseitiviy, m_rotationSlewrate);
  }

  public double getHeading(){
    if (getRawHeadingMagnitude() <= m_headingDeadband) return m_headingLimiter.calculate(m_previousHeading,1e-6);
    m_previousHeading = m_headingLimiter.calculate(getRawHeadingAngle(), Functions.expoMS(getRawHeadingMagnitude(), m_headingExpo) * m_headingSenseitiviy);
    return m_previousHeading;
  }

  public Drivetrain getDrivetrain(){
    return m_drive;
  }

  public void setupShuffleboard(){
    if (!m_shuffleboardUpdates) return;
    
    m_translationalSenseitivityEntry = m_controllerTab.add("translationalSenseitivity",OIConstants.kTranslationalSenseitivity).getEntry();
    m_translationalExpoEntry = m_controllerTab.add("translationalExpo",OIConstants.kTranslationalExpo).getEntry();
    m_translationalDeadbandEntry = m_controllerTab.add("translationalDeadband",OIConstants.kTranslationalDeadband).getEntry();
    m_translationalSlewrateEntry = m_controllerTab.add("translationalSlewrate",OIConstants.kTranslationalSlewrate).getEntry();
    m_rotationSenseitiviyEntry = m_controllerTab.add("rotationSenseitiviy",OIConstants.kRotationSenseitiviy).getEntry();
    m_rotationExpoEntry = m_controllerTab.add("rotationExpo",OIConstants.kRotationExpo).getEntry();
    m_rotationDeadbandEntry = m_controllerTab.add("rotationDeadband",OIConstants.kRotationDeadband).getEntry();
    m_rotationSlewrateEntry = m_controllerTab.add("rotationSlewrate",OIConstants.kRotationSlewrate).getEntry();
    m_headingSenseitiviyEntry = m_controllerTab.add("headingSenseitiviy",OIConstants.kHeadingSenseitiviy).getEntry();
    m_headingExpoEntry = m_controllerTab.add("headingExpo",OIConstants.kHeadingExpo).getEntry();
    m_headingDeadbandEntry = m_controllerTab.add("headingDeadband",OIConstants.kHeadingDeadband).getEntry();
  }

  public void updateSettings(){ //updates the shuffleboard data
    if (!m_shuffleboardUpdates) return;

    m_translationalSenseitivity = m_translationalSenseitivityEntry.getDouble(OIConstants.kTranslationalSenseitivity);
    m_translationalExpo = m_translationalExpoEntry.getDouble(OIConstants.kTranslationalExpo);
    m_translationalDeadband = m_translationalDeadbandEntry.getDouble(OIConstants.kTranslationalDeadband);
    m_translationalSlewrate = m_translationalSlewrateEntry.getDouble(OIConstants.kTranslationalSlewrate);

    m_rotationSenseitiviy = m_rotationSenseitiviyEntry.getDouble(OIConstants.kRotationSenseitiviy);
    m_rotationExpo = m_rotationExpoEntry.getDouble(OIConstants.kRotationExpo);
    m_rotationDeadband = m_rotationDeadbandEntry.getDouble(OIConstants.kRotationDeadband);
    m_rotationSlewrate = m_rotationSlewrateEntry.getDouble(OIConstants.kRotationSlewrate);

    m_headingSenseitiviy = m_headingSenseitiviyEntry.getDouble(OIConstants.kHeadingSenseitiviy);
    m_headingExpo = m_headingExpoEntry.getDouble(OIConstants.kHeadingExpo);
    m_headingDeadband = m_headingDeadbandEntry.getDouble(OIConstants.kHeadingDeadband);
  }

  public abstract void configureControls();
  public abstract double getRawSideTranslation();
  public abstract double getRawForwardTranslation();
  public abstract double getRawRotation();
  public abstract double getRawHeadingAngle();
  public abstract double getRawHeadingMagnitude();
}