package frc.robot.controls;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.SetFormationX;
import frc.robot.constants.Constants;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.OIConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.DynamicSlewRateLimiter;
import frc.robot.util.Functions;
import lib.controllers.Controller;
import lib.controllers.Ex3DProController;
import lib.controllers.Ex3DProController.Ex3DProAxis;
import lib.controllers.Ex3DProController.Ex3DProButton;
import lib.controllers.GameController;
import lib.controllers.GameController.GCAxis;
import lib.controllers.GameController.GCButton;
import lib.controllers.MadCatzController;
import lib.controllers.MadCatzController.MadCatzAxis;
import lib.controllers.MadCatzController.MadCatzButton;

public class Driver {
  private static GameController driverGC = new GameController(OIConstants.kDriverJoy);
  private static Ex3DProController driverEPC = new Ex3DProController(2);
  private static MadCatzController driverMCC = new MadCatzController(3);
  private static Controller m_controllerType = new GameController(-1);

  private static DynamicSlewRateLimiter m_xspeedLimiter = new DynamicSlewRateLimiter(OIConstants.kTranslationalSlewrate);
  private static DynamicSlewRateLimiter m_yspeedLimiter = new DynamicSlewRateLimiter(OIConstants.kTranslationalSlewrate);
  private static DynamicSlewRateLimiter m_rotLimiter = new DynamicSlewRateLimiter(OIConstants.kRotationSlewrate);
  private static DynamicSlewRateLimiter m_headingLimiter = new DynamicSlewRateLimiter(OIConstants.kHeadingSensitivity);

  private static boolean m_invert;

  public static double m_previousHeading;

  //controller input changing
  private static GenericEntry m_translationalSensitivityEntry, m_translationalExpoEntry, m_translationalDeadbandEntry, m_translationalSlewrateEntry, m_fieldRelativeEntry;
  private static GenericEntry m_rotationSensitivityEntry, m_rotationExpoEntry, m_rotationDeadbandEntry, m_rotationSlewrateEntry;
  private static GenericEntry m_headingSensitivityEntry, m_headingExpoEntry, m_headingDeadbandEntry, m_consantHeadingMagnitudeEntry;
  private static GenericEntry m_invertEntry;
  private static SendableChooser<Controller> m_controllerTypeChooser = new SendableChooser<>();

  /**
   * Configures all the driver controls, which are the default controls for the robot.
   */
  public static void configureControls(Drivetrain drive) {
    ShuffleboardTab controllerTab = Shuffleboard.getTab("Controller");

    controllerTab.add("Controller Type", m_controllerType);

    m_translationalSensitivityEntry = controllerTab.add("translationalSenseitivity", OIConstants.kTranslationalSenseitivity).getEntry();
    m_translationalExpoEntry = controllerTab.add("translationalExpo", OIConstants.kTranslationalExpo).getEntry();
    m_translationalDeadbandEntry = controllerTab.add("translationalDeadband", OIConstants.kTranslationalDeadband).getEntry();
    m_translationalSlewrateEntry = controllerTab.add("translationalSlewrate", OIConstants.kTranslationalSlewrate).getEntry();
    m_fieldRelativeEntry = controllerTab.add("Field Relitive", OIConstants.kFieldRelative).getEntry();

    m_rotationSensitivityEntry = controllerTab.add("rotationSenseitiviy", OIConstants.kRotationSenseitiviy).getEntry();
    m_rotationExpoEntry = controllerTab.add("rotationExpo", OIConstants.kRotationExpo).getEntry();
    m_rotationDeadbandEntry = controllerTab.add("rotationDeadband", OIConstants.kRotationDeadband).getEntry();
    m_rotationSlewrateEntry = controllerTab.add("rotationSlewrate", OIConstants.kRotationSlewrate).getEntry();

    m_headingSensitivityEntry = controllerTab.add("headingSenseitiviy", OIConstants.kHeadingSensitivity).getEntry();
    m_headingExpoEntry = controllerTab.add("headingExpo", OIConstants.kHeadingExpo).getEntry();
    m_headingDeadbandEntry = controllerTab.add("headingDeadband", OIConstants.kHeadingDeadband).getEntry();
    
    m_headingLimiter.setContinuousLimits(-Math.PI,Math.PI);
    m_headingLimiter.enableContinuous(true);

    driverGC.get(GCButton.A).whileTrue(new SetFormationX(drive));
    driverEPC.get(Ex3DProButton.B1).whileTrue(new SetFormationX(drive));
    driverMCC.get(MadCatzButton.B1).whileTrue(new SetFormationX(drive));

    
    driverGC.get(GCButton.START).onTrue(new InstantCommand(() -> drive.setPigeonYaw(DriveConstants.kStartingHeadingDegrees)));
    driverEPC.get(Ex3DProButton.B2).whileTrue(new InstantCommand(() -> drive.setPigeonYaw(DriveConstants.kStartingHeadingDegrees)));
    driverMCC.get(MadCatzButton.B2).whileTrue(new InstantCommand(() -> drive.setPigeonYaw(DriveConstants.kStartingHeadingDegrees)));
  }

  public static double getForwardTranslation() {
    return m_yspeedLimiter.calculate(-Functions.expoMS(Functions.deadband(getRawForwardTranslation(), getTranslationDeadband()), getTranslationExpo()) * DriveConstants.kMaxSpeed * getTranslationSensitivity(), getTranslationSlewrate());
  }
  public static double getSideTranslation() {
    return m_xspeedLimiter.calculate(-Functions.expoMS(Functions.deadband(getRawSideTranslation(), getTranslationDeadband()), getTranslationExpo()) * DriveConstants.kMaxSpeed * getTranslationSensitivity(), getTranslationSlewrate());
  }
  public static double getRotation() {
    return m_rotLimiter.calculate(-Functions.expoMS(Functions.deadband(getRawRotation(), getRotationDeadband()), getRotationExpo()) * DriveConstants.kMaxAngularSpeed * getRotationSensitivity(), getRotationSlewrate());
  }
  public static double getHeading() {
    if (getRawHeadingMagnitude() <= getHeadingDeadband()) return m_headingLimiter.calculate(m_previousHeading,1e-6);
    m_previousHeading = m_headingLimiter.calculate(getRawHeadingAngle(), Functions.expoMS(getRawHeadingMagnitude(), getHeadingExpo())); //TODO heading sensitivity used to be here
    // if (getRawHeadingMagnitude() <= m_headingDeadband) return m_previousHeading;
    // m_previousHeading = getRawHeadingAngle();
    
    // System.out.println("heading: " + m_previousHeading);
    return m_previousHeading;
    // return 0;
  }
  
/*
  public double getForwardTranslationSlew() {
    return -m_xspeedLimiter.calculate(Functions.deadband(getRawLeftY(), OIConstants.kDeadband)) * DriveConstants.kMaxSpeed * 0.1;
  }

  public double getSideTranslationSlew() {
    return -m_yspeedLimiter.calculate(Functions.deadband(getRawLeftX(), OIConstants.kDeadband)) * DriveConstants.kMaxSpeed * 0.1;
  }

  public double getRotationSlew() {
    return -m_rotLimiter.calculate(Functions.deadband(getRawRightX(), OIConstants.kDeadband)) * DriveConstants.kMaxAngularSpeed * 0.1;
  }
*/

  public static double getRawSideTranslation() {
    if (m_controllerType instanceof GameController && m_invert) return driverGC.get(GCAxis.RIGHT_X);
    if (m_controllerType instanceof GameController) return driverGC.get(GCAxis.LEFT_X);
    if (m_controllerType instanceof Ex3DProController) return -driverEPC.get(Ex3DProAxis.X);
    if (m_controllerType instanceof MadCatzController) return driverMCC.get(MadCatzAxis.X);
    return 0;
  }
  public static double getRawForwardTranslation() {
    if (m_controllerType instanceof GameController && m_invert) return driverGC.get(GCAxis.RIGHT_Y);
    if (m_controllerType instanceof GameController) return driverGC.get(GCAxis.LEFT_Y);
    if (m_controllerType instanceof Ex3DProController) return -driverEPC.get(Ex3DProAxis.Y);
    if (m_controllerType instanceof MadCatzController) return driverMCC.get(MadCatzAxis.Y);
    return 0;
  }

  public static double getRawRotation() {
    if (m_controllerType instanceof GameController && m_invert) return driverGC.get(GCAxis.LEFT_X);
    if (m_controllerType instanceof GameController) return driverGC.get(GCAxis.RIGHT_X);
    if (m_controllerType instanceof Ex3DProController) return driverEPC.get(Ex3DProAxis.Z);
    if (m_controllerType instanceof MadCatzController) return driverMCC.get(MadCatzAxis.ZROTATE);
    return 0;
  }
  public static double getRawHeadingAngle() {
    if (m_controllerType instanceof GameController && m_invert) return Functions.calculateAngle(driverGC.get(GCAxis.LEFT_X),-driverGC.get(GCAxis.LEFT_Y))-Math.PI/2;
    if (m_controllerType instanceof GameController) return Functions.calculateAngle(driverGC.get(GCAxis.RIGHT_X),-driverGC.get(GCAxis.RIGHT_Y))-Math.PI/2;
    if (m_controllerType instanceof Ex3DProController) return driverEPC.get(Ex3DProAxis.Z) * Math.PI;
    if (m_controllerType instanceof MadCatzController) return driverMCC.get(MadCatzAxis.ZROTATE) * Math.PI;
    return 0;
  }
  public static double getRawHeadingMagnitude() {
    if (getConsantHeadingMagnitude()) return 1;
    if (m_controllerType instanceof GameController && m_invert) return Functions.calculateHypotenuse(driverGC.get(GCAxis.LEFT_X),driverGC.get(GCAxis.LEFT_Y));
    if (m_controllerType instanceof GameController) return Functions.calculateHypotenuse(driverGC.get(GCAxis.RIGHT_X),driverGC.get(GCAxis.RIGHT_Y));
    if (m_controllerType instanceof Ex3DProController) return driverEPC.get(Ex3DProAxis.SLIDER);
    if (m_controllerType instanceof MadCatzController) return driverMCC.get(MadCatzAxis.SLIDER);
    return 0;
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
  public static double getRotationSenseitiviy(){
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
  public static double getRotationSensitivity(){
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
