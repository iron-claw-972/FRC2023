package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.ModuleConstants;
import frc.robot.util.LogManager;

/** Represents a swerve drive style drivetrain.
 * Module IDs are:
 * 1: Front left
 * 2: Front right
 * 3: Back left
 * 4: Back right
*/
public class Drivetrain extends SubsystemBase {

  // Swerve modules and other
  public SwerveModuleState[] m_swerveModuleStates = new SwerveModuleState[] {
    new SwerveModuleState(),
    new SwerveModuleState(),
    new SwerveModuleState(),
    new SwerveModuleState()
  };

  //TODO: not m_ if it is public. Also should this be public?
  public final Module[] m_modules;

  public final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
    new Translation2d(DriveConstants.kTrackWidth / 2, DriveConstants.kTrackWidth / 2),
    new Translation2d(DriveConstants.kTrackWidth / 2, -DriveConstants.kTrackWidth / 2),
    new Translation2d(-DriveConstants.kTrackWidth / 2, DriveConstants.kTrackWidth / 2),
    new Translation2d(-DriveConstants.kTrackWidth / 2, -DriveConstants.kTrackWidth / 2)
  );

  // Pigeon
  private final WPI_Pigeon2 m_pigeon = new WPI_Pigeon2(DriveConstants.kPigeon, Constants.kCanivoreCAN);
  private boolean m_hasResetYaw = false; // the initial yaw has been set 

  public double m_headingPIDOutput = 0;

  // Odometry
  private final SwerveDriveOdometry m_odometry;
  private Pose2d m_robotPose = new Pose2d();

  // Displays the field with the robots estimated pose on it
  private final Field2d m_fieldDisplay = new Field2d();
  
  // PID Controllers
  private PIDController m_xController = new PIDController(0,0,0);
  private PIDController m_yController = new PIDController(0, 0, 0);
  private PIDController m_rotationController = new PIDController(DriveConstants.KheadingP, DriveConstants.KheadingI, DriveConstants.KheadingD);

  public Drivetrain() {
      m_modules = new Module[]{
        Module.create(ModuleConstants.TEST_FL),
        Module.create(ModuleConstants.TEST_FR),
        Module.create(ModuleConstants.TEST_BL),
        Module.create(ModuleConstants.TEST_BR)
      };

    m_odometry = new SwerveDriveOdometry(m_kinematics, m_pigeon.getRotation2d(), getModulePositions(), m_robotPose);
    m_rotationController.enableContinuousInput(-Math.PI,Math.PI);
    DoubleSupplier[] poseSupplier = {() -> getPose().getX(), () -> getPose().getY(), () -> getPose().getRotation().getRadians()};
    LogManager.addDoubleArray("Pose2d", poseSupplier);

    m_fieldDisplay.setRobotPose(getPose());
    SmartDashboard.putData("Field", m_fieldDisplay);
  }

  @Override
  public void periodic() {
    if (!Robot.isReal()) {
      for (int i = 0; i < m_modules.length; i++) {
        m_modules[i].periodic();
      }
    }
    updateOdometry();
    
    m_fieldDisplay.setRobotPose(getPose());
  }

  public void runChassisPID(double x, double y, double rot) {
    double xSpeed = m_xController.calculate(m_odometry.getPoseMeters().getX(), x);
    double ySpeed = m_yController.calculate(m_odometry.getPoseMeters().getY(), y);
    double rotRadians = m_rotationController.calculate(getAngleHeading(), rot);
    System.out.println(rotRadians);
    driveRot(xSpeed, ySpeed, rotRadians, true);
  }

  public void setPigeonYaw(double degrees) {
    m_pigeon.setYaw(degrees);
  }

  /**
   * Resets the pigeon yaw, but only if it hasn't already been reset. Will reset it to {@link DriveConstants.kStartingHeadingDegrees}
   */
  public void initializePigeonYaw(boolean force) {
    if (!m_hasResetYaw || force) {
      m_hasResetYaw = true;
      setPigeonYaw(DriveConstants.kStartingHeadingDegrees);
    }
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed speed of the robot in the x direction (forward)
   * @param ySpeed speed of the robot in the y direction (sideways)
   * @param rot angular rate of the robot
   * @param fieldRelative whether the provided x and y speeds are relative to the field
   */
  public void driveRot(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {

    // TODO: Fix Swerve drive sim
    if (!Robot.isReal()) {
      System.out.println(xSpeed + " " + ySpeed + " " + rot);
      m_pigeon.getSimCollection().addHeading(rot / (2 * Math.PI));
    }

    m_swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_pigeon.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(m_swerveModuleStates, DriveConstants.kMaxSpeed);
    setModuleStates(m_swerveModuleStates);
  }

  public void driveHeading(double xSpeed, double ySpeed, double heading, boolean fieldRelative) {
    m_headingPIDOutput = m_rotationController.calculate(getAngleHeading(),heading);
    double rot = m_headingPIDOutput;

    // TODO: Fix Swerve drive sim
    if (!Robot.isReal()) {
      System.out.println(xSpeed + " " + ySpeed + " " + rot);
      m_pigeon.getSimCollection().addHeading(rot / (2 * Math.PI));
    }

    m_swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_pigeon.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(m_swerveModuleStates, DriveConstants.kMaxSpeed);
    setModuleStates(m_swerveModuleStates);
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_robotPose = m_odometry.update(
      m_pigeon.getRotation2d(),
      getModulePositions()
    );
  }

  /**
   * Returns the angular rate from the pigeon.
   * 
   * @param id 0 for x, 1 for y, 2 for z
   * @return the rate in rads/s from the pigeon
   */
  public double getAngularRate(int id) {
    double[] rawGyros = new double[3];
    m_pigeon.getRawGyro(rawGyros);
    return rawGyros[id] * Math.PI / 180;
  }

  /**
   * Gets the current robot pose from the odometry.
   */
  public Pose2d getPose() {
    return m_robotPose;
  }

  /**
   * Resets the odometry to the given pose and gyro angle.
   * @param pose current robot pose
   * @param gyroAngle current robot gyro angle
   */
  public void resetOdometry(Pose2d pose, Rotation2d gyroAngle){
    m_odometry.resetPosition(gyroAngle, getModulePositions(), pose);
  }

  /**
   * @return the pidgeon's Rotation2d
   */
  public Rotation2d getRotation2d(){
    return m_pigeon.getRotation2d(); 
  }

  /**
   * Gets the angle heading from the pigeon.
   * 
   * @return the heading angle in radians, from -pi to pi
   */
  public double getAngleHeading() {
    double angle = m_pigeon.getRotation2d().getRadians();
    return MathUtil.angleModulus(angle);
  }

  /**
   * Gets an array of all the swerve module positions.
   * 
   * @return an array of all swerve module positions
   */
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[]{
      new SwerveModulePosition(m_modules[0].getDrivePosition(), m_modules[0].getState().angle),
      new SwerveModulePosition(m_modules[1].getDrivePosition(), m_modules[1].getState().angle),
      new SwerveModulePosition(m_modules[2].getDrivePosition(), m_modules[2].getState().angle),
      new SwerveModulePosition(m_modules[3].getDrivePosition(), m_modules[3].getState().angle)
      // new SwerveModulePosition(m_modules[0].getDrivePosition(), Rotation2d.fromDegrees(m_modules[0].getAngle())),
      // new SwerveModulePosition(m_modules[1].getDrivePosition(), Rotation2d.fromDegrees(m_modules[1].getAngle())),
      // new SwerveModulePosition(m_modules[2].getDrivePosition(), Rotation2d.fromDegrees(m_modules[2].getAngle())),
      // new SwerveModulePosition(m_modules[3].getDrivePosition(), Rotation2d.fromDegrees(m_modules[3].getAngle()))
    };
    return positions;
  }

  /**
   * Sets the desired states for all swerve modules.
   * 
   * @param swerveModuleStates an array of module states to set swerve modules to. Order of the array matters here!
   */
  public void setModuleStates(SwerveModuleState[] swerveModuleStates) {
    for (int i = 0; i < 4; i++) {
      m_modules[i].setDesiredState(swerveModuleStates[i]);
    }
  }

  public void stop(){
    for (int i = 0; i < 4; i++) {
      m_modules[i].setDriveVoltage(0);
      m_modules[i].setSteerVoltage(0);
    }
  }

  public void driveVoltsTest(double volts) {
    // setAllOptimize(false);
    for (int i = 0; i < 4; i++) {
      m_modules[i].setDriveVoltage(volts);
    }
    m_modules[0].setSteerAngle(new Rotation2d(Units.degreesToRadians(135)));
    m_modules[1].setSteerAngle(new Rotation2d(Units.degreesToRadians(45)));
    m_modules[2].setSteerAngle(new Rotation2d(Units.degreesToRadians(225)));
    m_modules[3].setSteerAngle(new Rotation2d(Units.degreesToRadians(315)));
  }
  public void steerVoltsTest(double volts) {
    // setAllOptimize(false);
    for (int i = 0; i < 4; i++) {
      m_modules[i].setDriveVoltage(0);
      m_modules[i].setSteerVoltage(volts);
    }
  }
  public boolean isDriveVelocityAcurate(){
    return 
      Math.abs(m_modules[0].getDriveVelocityError()) < 0.1 &&
      Math.abs(m_modules[1].getDriveVelocityError()) < 0.1 &&
      Math.abs(m_modules[2].getDriveVelocityError()) < 0.1 &&
      Math.abs(m_modules[3].getDriveVelocityError()) < 0.1;
  }

  public boolean isSteerAngleAcurate(){
    return 
      Math.abs(m_modules[0].getSteerAngleError()) < Units.degreesToRadians(1) &&
      Math.abs(m_modules[1].getSteerAngleError()) < Units.degreesToRadians(1) &&
      Math.abs(m_modules[2].getSteerAngleError()) < Units.degreesToRadians(1) &&
      Math.abs(m_modules[3].getSteerAngleError()) < Units.degreesToRadians(1);
  }
  

  public PIDController getXController() {
      return m_xController;
  }
  public PIDController getYController() {
      return m_yController;
  }
  public PIDController getRotationController() {
    return m_rotationController;
  }
  public void setAllOptimize(Boolean optimizeSate){
    for (int i = 0; i < 4; i++) {
      m_modules[i].setOptimize(optimizeSate);
    }
  }

}