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
  
  private static final double kRotationSensitivity = 0.2;
  private static final double kRotationExpo = 4;
  private static final double kRotationDeadband = 0.01;
  private static final double kRotationSlewrate = 10;

  private static final double kHeadingSensitivity = 4;
  private static final double kHeadingExpo = 2;
  private static final double kHeadingDeadband = 0.05;
  private static final boolean kConstantHeadingMagnitude = false;

  private static GenericEntry translationalSensitivityEntry, translationalExpoEntry, translationalDeadbandEntry, translationalSlewrateEntry;
  private static GenericEntry rotationSensitivityEntry, rotationExpoEntry, rotationDeadbandEntry, rotationSlewrateEntry;
  private static GenericEntry headingSensitivityEntry, headingExpoEntry, headingDeadbandEntry;

  private static double translationalSensitivity = kTranslationalSensitivity;
  private static double translationalExpo = kTranslationalExpo;
  private static double translationalDeadband = kTranslationalDeadband;
  private static double translationalSlewrate = kTranslationalSlewrate;

  private static double rotationSensitivity = kRotationSensitivity;
  private static double rotationExpo = kRotationExpo;
  private static double rotationDeadband = kRotationDeadband;
  private static double rotationSlewrate = kRotationSlewrate;

  private static double headingSensitivity = kHeadingSensitivity;
  private static double headingExpo = kHeadingExpo;
  private static double headingDeadband = kHeadingDeadband;
  private static double previousHeading = 0;

  private static final DynamicSlewRateLimiter xSpeedLimiter = new DynamicSlewRateLimiter(translationalSlewrate);
  private static final DynamicSlewRateLimiter ySpeedLimiter = new DynamicSlewRateLimiter(translationalSlewrate);
  private static final DynamicSlewRateLimiter rotLimiter = new DynamicSlewRateLimiter(rotationSlewrate);
  private static final DynamicSlewRateLimiter headingLimiter = new DynamicSlewRateLimiter(headingSensitivity);

  private static final int kDriverJoy = 0;
  private static final int kOperatorJoy = 1;
  private static final int kManualJoy = 2;
  private static final int kTestJoy = 3;

  private static GameController driver;
  private static GameController operator;
  private static GameController manual;
  private static GameController test;

  public static void configure(Drivetrain drive, Intake intake, FourBarArm arm) {
    configureDriverControls(drive);
    configureOperatorControls(intake, arm);
    configureManualControls(intake, arm);
    configureTestControls(intake, arm);
  }

  // DRIVER
  public static void configureDriverControls(Drivetrain drive) {
    driver = new GameController(kDriverJoy);

    headingLimiter.setContinuousLimits(-Math.PI,Math.PI);
    headingLimiter.enableContinuous(true);

    driver.START.onTrue(new InstantCommand(() -> drive.setPigeonYaw(Constants.Drive.kStartingHeadingDegrees)));
    driver.A.whileTrue(new SetFormationX(drive));
  }

  // OPERATOR
  public static void configureOperatorControls(Intake intake, FourBarArm arm) {
    operator = new GameController(kOperatorJoy);
  }

  // MANUAL
  public static void configureManualControls(Intake intake, FourBarArm arm) {
    manual = new GameController(kManualJoy);

    manual.DPAD_DOWN.onTrue(new InstantCommand(() -> intake.intake(intake.kIntakeSpeed), intake));
    manual.DPAD_UP.onTrue(new InstantCommand(() -> intake.intake(intake.kOuttakeSpeed),intake));
    manual.DPAD_LEFT.onTrue(new InstantCommand(() -> intake.stop(), intake));
    
    manual.Y.onTrue(new InstantCommand(() -> arm.setMotorPower(0.05)));
    manual.X.onTrue(new InstantCommand(() -> arm.setMotorPower(-0.05)));
    manual.B.onTrue(new InstantCommand(() -> arm.setMotorPower(0)));
  }

  // TEST
  public static void configureTestControls(Intake intake, FourBarArm arm) {
    test = new GameController(kTestJoy);

    // elevator controls
    test.Y.onTrue(new ExtendToPosition(arm, arm.kTopPosition));
    test.X.onTrue(new ExtendToPosition(arm, arm.kMiddlePosition));
    test.A.onTrue(new ExtendToPosition(arm, arm.kIntakePosition));
    test.B.onTrue(new ExtendToPosition(arm, arm.kShelfPosition));
    
    // intake controls
    test.DPAD_DOWN.onTrue(new InstantCommand(() -> intake.intake(intake.kIntakeSpeed), intake));
    test.DPAD_UP.onTrue(new InstantCommand(() -> intake.intake(intake.kOuttakeSpeed),intake));
    test.DPAD_LEFT.onTrue(new InstantCommand(() -> intake.stop(), intake));
  }

  public static double getRawSideTranslation() { 
    return driver.LEFT_X();
  }
  
  public static double getRawForwardTranslation() {
    return driver.LEFT_Y();
  }
  
  public static double getRawRotation() { 
    return driver.RIGHT_X();
  }
  
  public static double getRawHeadingAngle() { 
    return Functions.calculateAngle(driver.RIGHT_X(), -driver.RIGHT_Y()) - Math.PI/2;
  }
  
  public static double getRawHeadingMagnitude() { 
    return Functions.calculateHypotenuse(driver.RIGHT_X(), driver.RIGHT_Y());
  }

  public static double getForwardTranslation() {
    return ySpeedLimiter.calculate(-Functions.expoMS(Functions.deadband(getRawForwardTranslation(), translationalDeadband), translationalExpo) * Constants.Drive.kMaxSpeed * translationalSensitivity, translationalSlewrate);
  }

  public static double getSideTranslation() {
    return xSpeedLimiter.calculate(-Functions.expoMS(Functions.deadband(getRawSideTranslation(), translationalDeadband), translationalExpo) * Constants.Drive.kMaxSpeed * translationalSensitivity, translationalSlewrate);
  }
  
  public static double getRotation() {
    return rotLimiter.calculate(-Functions.expoMS(Functions.deadband(getRawRotation(), rotationDeadband), rotationExpo) * Constants.Drive.kMaxAngularSpeed * rotationSensitivity, rotationSlewrate);
  }

  public static double getHeading() {
    if (getRawHeadingMagnitude() <= headingDeadband) return headingLimiter.calculate(previousHeading,1e-6);
    previousHeading = headingLimiter.calculate(getRawHeadingAngle(), Functions.expoMS(getRawHeadingMagnitude(), headingExpo) * headingSensitivity);
    return previousHeading;
  }

  public static void setupShuffleboard(ShuffleboardTab tab) {
    translationalSensitivityEntry = tab.add("translationalSensitivity", kTranslationalSensitivity).getEntry();
    translationalExpoEntry = tab.add("translationalExpo", kTranslationalExpo).getEntry();
    translationalDeadbandEntry = tab.add("translationalDeadband", kTranslationalDeadband).getEntry();
    translationalSlewrateEntry = tab.add("translationalSlewrate", kTranslationalSlewrate).getEntry();
    rotationSensitivityEntry = tab.add("rotationSensitivity", kRotationSensitivity).getEntry();
    rotationExpoEntry = tab.add("rotationExpo", kRotationExpo).getEntry();
    rotationDeadbandEntry = tab.add("rotationDeadband", kRotationDeadband).getEntry();
    rotationSlewrateEntry = tab.add("rotationSlewrate", kRotationSlewrate).getEntry();
    headingSensitivityEntry = tab.add("headingSensitivity", kHeadingSensitivity).getEntry();
    headingExpoEntry = tab.add("headingExpo", kHeadingExpo).getEntry();
    headingDeadbandEntry = tab.add("headingDeadband", kHeadingDeadband).getEntry();
  }

  public static void updateShuffleboard() {
    translationalSensitivity = translationalSensitivityEntry.getDouble(kTranslationalSensitivity);
    translationalExpo = translationalExpoEntry.getDouble(kTranslationalExpo);
    translationalDeadband = translationalDeadbandEntry.getDouble(kTranslationalDeadband);
    translationalSlewrate = translationalSlewrateEntry.getDouble(kTranslationalSlewrate);

    rotationSensitivity = rotationSensitivityEntry.getDouble(kRotationSensitivity);
    rotationExpo = rotationExpoEntry.getDouble(kRotationExpo);
    rotationDeadband = rotationDeadbandEntry.getDouble(kRotationDeadband);
    rotationSlewrate = rotationSlewrateEntry.getDouble(kRotationSlewrate);

    headingSensitivity = headingSensitivityEntry.getDouble(kHeadingSensitivity);
    headingExpo = headingExpoEntry.getDouble(kHeadingExpo);
    headingDeadband = headingDeadbandEntry.getDouble(kHeadingDeadband);
  }
}
