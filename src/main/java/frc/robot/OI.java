package frc.robot;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.arm.ExtendToPosition;
import frc.robot.commands.drive.SetFormationX;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.FourBarArm;
import frc.robot.subsystems.Intake;
import frc.robot.util.DynamicSlewRateLimiter;
import frc.robot.util.Functions;
import lib.controllers.GameController;

public class OI {
  private static final double kTranslationalSensitivity = 0.3;
  private static final double kTranslationalExpo = 2;
  private static final double kTranslationalDeadband = 0.05;
  private static final double kTranslationalSlewrate = 4;
  private static final boolean kFieldRelative = true;
  
  private static final double kRotationSensitivity = 0.05;
  private static final double kRotationExpo = 4;
  private static final double kRotationDeadband = 0.01;
  private static final double kRotationSlewrate = 10;

  private static final double kHeadingSensitivity = 4;
  private static final double kHeadingExpo = 2;
  private static final double kHeadingDeadband = 0.05;
  private static final boolean kConstantHeadingMagnitude = false;

  private static GenericEntry m_translationalSensitivityEntry, m_translationalExpoEntry, m_translationalDeadbandEntry, m_translationalSlewrateEntry;
  private static GenericEntry m_rotationSensitivityEntry, m_rotationExpoEntry, m_rotationDeadbandEntry, m_rotationSlewrateEntry;
  private static GenericEntry m_headingSensitivityEntry, m_headingExpoEntry, m_headingDeadbandEntry;

  private static double m_translationalSensitivity = kTranslationalSensitivity;
  private static double m_translationalExpo = kTranslationalExpo;
  private static double m_translationalDeadband = kTranslationalDeadband;
  private static double m_translationalSlewrate = kTranslationalSlewrate;

  private static double m_rotationSensitivity = kRotationSensitivity;
  private static double m_rotationExpo = kRotationExpo;
  private static double m_rotationDeadband = kRotationDeadband;
  private static double m_rotationSlewrate = kRotationSlewrate;

  private static double m_headingSensitivity = kHeadingSensitivity;
  private static double m_headingExpo = kHeadingExpo;
  private static double m_headingDeadband = kHeadingDeadband;
  private static double m_previousHeading = 0;

  private static final DynamicSlewRateLimiter m_xSpeedLimiter = new DynamicSlewRateLimiter(m_translationalSlewrate);
  private static final DynamicSlewRateLimiter m_ySpeedLimiter = new DynamicSlewRateLimiter(m_translationalSlewrate);
  private static final DynamicSlewRateLimiter m_rotLimiter = new DynamicSlewRateLimiter(m_rotationSlewrate);
  private static final DynamicSlewRateLimiter m_headingLimiter = new DynamicSlewRateLimiter(m_headingSensitivity);

  private static final int m_driverJoy = 0;
  private static final int kOperatorJoy = 1;
  private static final int kManualJoy = 2;
  private static final int kTestJoy = 3;

  private static GameController m_driver;
  private static GameController m_operator;
  private static GameController m_manual;
  private static GameController m_test;

  public static void configure(Drivetrain drive, Intake intake, FourBarArm arm) {
    m_driver = new GameController(m_driverJoy);
    m_operator = new GameController(kOperatorJoy);
    m_manual = new GameController(kManualJoy);
    m_test = new GameController(kTestJoy);

    configureDriverControls(drive);
    configureOperatorControls(intake, arm);
    configureManualControls(intake, arm);
    configureTestControls(intake, arm);
  }

  // DRIVER
  public static void configureDriverControls(Drivetrain drive) {
    m_headingLimiter.setContinuousLimits(-Math.PI,Math.PI);
    m_headingLimiter.enableContinuous(true);

    m_driver.START.onTrue(new InstantCommand(() -> drive.setPigeonYaw(Constants.Drive.kStartingHeadingDegrees)));
    m_driver.A.whileTrue(new SetFormationX(drive));
  }

  // OPERATOR
  public static void configureOperatorControls(Intake intake, FourBarArm arm) {

  }

  // MANUAL
  public static void configureManualControls(Intake intake, FourBarArm arm) {
    m_manual.DPAD_DOWN.onTrue(new InstantCommand(() -> intake.intake(intake.kIntakeSpeed), intake));
    m_manual.DPAD_UP.onTrue(new InstantCommand(() -> intake.intake(intake.kOuttakeSpeed),intake));
    m_manual.DPAD_LEFT.onTrue(new InstantCommand(() -> intake.stop(), intake));
    
    m_manual.Y.onTrue(new InstantCommand(() -> arm.setMotorPower(0.05)));
    m_manual.X.onTrue(new InstantCommand(() -> arm.setMotorPower(-0.05)));
    m_manual.B.onTrue(new InstantCommand(() -> arm.setMotorPower(0)));
  }

  // TEST
  public static void configureTestControls(Intake intake, FourBarArm arm) {
    // elevator controls
    m_test.Y.onTrue(new ExtendToPosition(arm, arm.kTopPosition));
    m_test.X.onTrue(new ExtendToPosition(arm, arm.kMiddlePosition));
    m_test.A.onTrue(new ExtendToPosition(arm, arm.kIntakePosition));
    m_test.B.onTrue(new ExtendToPosition(arm, arm.kShelfPosition));
    
    // intake controls
    m_test.DPAD_DOWN.onTrue(new InstantCommand(() -> intake.intake(intake.kIntakeSpeed), intake));
    m_test.DPAD_UP.onTrue(new InstantCommand(() -> intake.intake(intake.kOuttakeSpeed),intake));
    m_test.DPAD_LEFT.onTrue(new InstantCommand(() -> intake.stop(), intake));
  }

  public static double getRawSideTranslation() { 
    return m_driver.LEFT_X();
  }
  
  public static double getRawForwardTranslation() {
    return m_driver.LEFT_Y();
  }
  
  public static double getRawRotation() { 
    return m_driver.RIGHT_X();
  }
  
  public static double getRawHeadingAngle() { 
    return Functions.calculateAngle(m_driver.RIGHT_X(), -m_driver.RIGHT_Y()) - Math.PI/2;
  }
  
  public static double getRawHeadingMagnitude() { 
    return Functions.calculateHypotenuse(m_driver.RIGHT_X(), m_driver.RIGHT_Y());
  }

  public static double getForwardTranslation() {
    return m_ySpeedLimiter.calculate(-Functions.expoMS(Functions.deadband(getRawForwardTranslation(), m_translationalDeadband), m_translationalExpo) * Constants.Drive.kMaxSpeed * m_translationalSensitivity, m_translationalSlewrate);
  }

  public static double getSideTranslation() {
    return m_xSpeedLimiter.calculate(-Functions.expoMS(Functions.deadband(getRawSideTranslation(), m_translationalDeadband), m_translationalExpo) * Constants.Drive.kMaxSpeed * m_translationalSensitivity, m_translationalSlewrate);
  }
  
  public static double getRotation() {
    return m_rotLimiter.calculate(-Functions.expoMS(Functions.deadband(getRawRotation(), m_rotationDeadband), m_rotationExpo) * Constants.Drive.kMaxAngularSpeed * m_rotationSensitivity, m_rotationSlewrate);
  }

  public static double getHeading() {
    if (getRawHeadingMagnitude() <= m_headingDeadband) return m_headingLimiter.calculate(m_previousHeading,1e-6);
    m_previousHeading = m_headingLimiter.calculate(getRawHeadingAngle(), Functions.expoMS(getRawHeadingMagnitude(), m_headingExpo) * m_headingSensitivity);
    return m_previousHeading;
  }

  public static void setupShuffleboard(ShuffleboardTab tab) {
    m_translationalSensitivityEntry = tab.add("translationalSensitivity", kTranslationalSensitivity).getEntry();
    m_translationalExpoEntry = tab.add("translationalExpo", kTranslationalExpo).getEntry();
    m_translationalDeadbandEntry = tab.add("translationalDeadband", kTranslationalDeadband).getEntry();
    m_translationalSlewrateEntry = tab.add("translationalSlewrate", kTranslationalSlewrate).getEntry();
    m_rotationSensitivityEntry = tab.add("rotationSensitivity", kRotationSensitivity).getEntry();
    m_rotationExpoEntry = tab.add("rotationExpo", kRotationExpo).getEntry();
    m_rotationDeadbandEntry = tab.add("rotationDeadband", kRotationDeadband).getEntry();
    m_rotationSlewrateEntry = tab.add("rotationSlewrate", kRotationSlewrate).getEntry();
    m_headingSensitivityEntry = tab.add("headingSensitivity", kHeadingSensitivity).getEntry();
    m_headingExpoEntry = tab.add("headingExpo", kHeadingExpo).getEntry();
    m_headingDeadbandEntry = tab.add("headingDeadband", kHeadingDeadband).getEntry();
  }

  public static void updateShuffleboard() {
    m_translationalSensitivity = m_translationalSensitivityEntry.getDouble(kTranslationalSensitivity);
    m_translationalExpo = m_translationalExpoEntry.getDouble(kTranslationalExpo);
    m_translationalDeadband = m_translationalDeadbandEntry.getDouble(kTranslationalDeadband);
    m_translationalSlewrate = m_translationalSlewrateEntry.getDouble(kTranslationalSlewrate);

    m_rotationSensitivity = m_rotationSensitivityEntry.getDouble(kRotationSensitivity);
    m_rotationExpo = m_rotationExpoEntry.getDouble(kRotationExpo);
    m_rotationDeadband = m_rotationDeadbandEntry.getDouble(kRotationDeadband);
    m_rotationSlewrate = m_rotationSlewrateEntry.getDouble(kRotationSlewrate);

    m_headingSensitivity = m_headingSensitivityEntry.getDouble(kHeadingSensitivity);
    m_headingExpo = m_headingExpoEntry.getDouble(kHeadingExpo);
    m_headingDeadband = m_headingDeadbandEntry.getDouble(kHeadingDeadband);
  }
}
