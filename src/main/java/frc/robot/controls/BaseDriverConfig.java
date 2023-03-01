package frc.robot.controls;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.constants.OIConstants;
import frc.robot.constants.swerve.DriveConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.DynamicSlewRateLimiter;
import frc.robot.util.Functions;

/**
 * Abstract class for different controller types.
 */
public abstract class BaseDriverConfig {

  private final Drivetrain m_drive;

  private boolean m_shuffleboardUpdates = false;

  private final ShuffleboardTab m_controllerTab;
  private GenericEntry m_translationalSensitivityEntry, m_translationalExpoEntry, m_translationalDeadbandEntry, m_translationalSlewrateEntry;
  private GenericEntry m_rotationSensitivityEntry, m_rotationExpoEntry, m_rotationDeadbandEntry, m_rotationSlewrateEntry;
  private GenericEntry m_headingSensitivityEntry, m_headingExpoEntry, m_headingDeadbandEntry;

  private double m_translationalSensitivity = OIConstants.kTranslationalSensitivity;
  private double m_translationalExpo = OIConstants.kTranslationalExpo;
  private double m_translationalDeadband = OIConstants.kTranslationalDeadband;
  private double m_translationalSlewrate = OIConstants.kTranslationalSlewrate;

  private double m_rotationSensitivity = OIConstants.kRotationSensitivity;
  private double m_rotationExpo = OIConstants.kRotationExpo;
  private double m_rotationDeadband = OIConstants.kRotationDeadband;
  private double m_rotationSlewrate = OIConstants.kRotationSlewrate;

  private double m_headingSensitivity = OIConstants.kHeadingSensitivity;
  private double m_headingExpo = OIConstants.kHeadingExpo;
  private double m_headingDeadband = OIConstants.kHeadingDeadband;
  private double m_previousHeading = 0;

  private final DynamicSlewRateLimiter m_xSpeedLimiter = new DynamicSlewRateLimiter(m_translationalSlewrate);
  private final DynamicSlewRateLimiter m_ySpeedLimiter = new DynamicSlewRateLimiter(m_translationalSlewrate);
  private final DynamicSlewRateLimiter m_rotLimiter = new DynamicSlewRateLimiter(m_rotationSlewrate);
  private final DynamicSlewRateLimiter m_headingLimiter = new DynamicSlewRateLimiter(m_headingSensitivity);

  /**
   * @param drive the drivetrain instance
   * @param controllerTab the shuffleboard controller tab
   * @param shuffleboardUpdates whether or not to update the shuffleboard
   */
  public BaseDriverConfig(Drivetrain drive, ShuffleboardTab controllerTab, boolean shuffleboardUpdates) {
    m_headingLimiter.setContinuousLimits(-Math.PI,Math.PI);
    m_headingLimiter.enableContinuous(true);
    m_controllerTab = controllerTab;
    m_shuffleboardUpdates = shuffleboardUpdates;
    m_drive = drive;
  }

  public double getForwardTranslation() {
    return m_ySpeedLimiter.calculate(-Functions.expoMS(Functions.deadband(getRawForwardTranslation(), m_translationalDeadband), m_translationalExpo) * DriveConstants.kMaxSpeed * m_translationalSensitivity, m_translationalSlewrate);
  }

  public double getSideTranslation() {
    return m_xSpeedLimiter.calculate(-Functions.expoMS(Functions.deadband(getRawSideTranslation(), m_translationalDeadband), m_translationalExpo) * DriveConstants.kMaxSpeed * m_translationalSensitivity, m_translationalSlewrate);
  }
  
  public double getRotation() {
    return m_rotLimiter.calculate(Functions.expoMS(Functions.deadband(getRawRotation(), m_rotationDeadband), m_rotationExpo) * DriveConstants.kMaxAngularSpeed * m_rotationSensitivity, m_rotationSlewrate);
  }

  public double getHeading() {
    if (getRawHeadingMagnitude() <= m_headingDeadband) return m_headingLimiter.calculate(m_previousHeading,1e-6);
    m_previousHeading = m_headingLimiter.calculate(getRawHeadingAngle(), Functions.expoMS(getRawHeadingMagnitude(), m_headingExpo) * m_headingSensitivity);
    return m_previousHeading;
  }

  public Drivetrain getDrivetrain() {
    return m_drive;
  }

  /**
   * Sets up shuffleboard values for the controller.
   */
  public void setupShuffleboard() {
    if (!m_shuffleboardUpdates) return;
    
    m_translationalSensitivityEntry = m_controllerTab.add("translationalSensitivity", OIConstants.kTranslationalSensitivity).getEntry();
    m_translationalExpoEntry = m_controllerTab.add("translationalExpo", OIConstants.kTranslationalExpo).getEntry();
    m_translationalDeadbandEntry = m_controllerTab.add("translationalDeadband", OIConstants.kTranslationalDeadband).getEntry();
    m_translationalSlewrateEntry = m_controllerTab.add("translationalSlewrate", OIConstants.kTranslationalSlewrate).getEntry();
    m_rotationSensitivityEntry = m_controllerTab.add("rotationSensitivity", OIConstants.kRotationSensitivity).getEntry();
    m_rotationExpoEntry = m_controllerTab.add("rotationExpo", OIConstants.kRotationExpo).getEntry();
    m_rotationDeadbandEntry = m_controllerTab.add("rotationDeadband", OIConstants.kRotationDeadband).getEntry();
    m_rotationSlewrateEntry = m_controllerTab.add("rotationSlewrate", OIConstants.kRotationSlewrate).getEntry();
    m_headingSensitivityEntry = m_controllerTab.add("headingSensitivity", OIConstants.kHeadingSensitivity).getEntry();
    m_headingExpoEntry = m_controllerTab.add("headingExpo", OIConstants.kHeadingExpo).getEntry();
    m_headingDeadbandEntry = m_controllerTab.add("headingDeadband", OIConstants.kHeadingDeadband).getEntry();
  }

  /**
   * Updates the controller settings from shuffleboard.
   */
  public void updateSettings() { //updates the shuffleboard data
    if (!m_shuffleboardUpdates) return;

    m_translationalSensitivity = m_translationalSensitivityEntry.getDouble(OIConstants.kTranslationalSensitivity);
    m_translationalExpo = m_translationalExpoEntry.getDouble(OIConstants.kTranslationalExpo);
    m_translationalDeadband = m_translationalDeadbandEntry.getDouble(OIConstants.kTranslationalDeadband);
    m_translationalSlewrate = m_translationalSlewrateEntry.getDouble(OIConstants.kTranslationalSlewrate);

    m_rotationSensitivity = m_rotationSensitivityEntry.getDouble(OIConstants.kRotationSensitivity);
    m_rotationExpo = m_rotationExpoEntry.getDouble(OIConstants.kRotationExpo);
    m_rotationDeadband = m_rotationDeadbandEntry.getDouble(OIConstants.kRotationDeadband);
    m_rotationSlewrate = m_rotationSlewrateEntry.getDouble(OIConstants.kRotationSlewrate);

    m_headingSensitivity = m_headingSensitivityEntry.getDouble(OIConstants.kHeadingSensitivity);
    m_headingExpo = m_headingExpoEntry.getDouble(OIConstants.kHeadingExpo);
    m_headingDeadband = m_headingDeadbandEntry.getDouble(OIConstants.kHeadingDeadband);
  }

  /**
   * Configures the controls for the controller.
   */
  public abstract void configureControls();

  public abstract double getRawSideTranslation();
  public abstract double getRawForwardTranslation();
  public abstract double getRawRotation();
  public abstract double getRawHeadingAngle();
  public abstract double getRawHeadingMagnitude();
}