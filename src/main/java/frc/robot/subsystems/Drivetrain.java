package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;
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
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.constants.ModuleConstants;
import frc.robot.constants.DriveConstants.TestDriveConstants;
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
    new Translation2d(Constants.drive.kTrackWidth / 2, Constants.drive.kTrackWidth / 2),
    new Translation2d(Constants.drive.kTrackWidth / 2, -Constants.drive.kTrackWidth / 2),
    new Translation2d(-Constants.drive.kTrackWidth / 2, Constants.drive.kTrackWidth / 2),
    new Translation2d(-Constants.drive.kTrackWidth / 2, -Constants.drive.kTrackWidth / 2)
  );

  // Pigeon
  private final WPI_Pigeon2 m_pigeon = new WPI_Pigeon2(Constants.drive.kPigeon, Constants.kCanivoreCAN);
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
  private PIDController m_rotationController = new PIDController(Constants.drive.KheadingP, Constants.drive.KheadingI, Constants.drive.KheadingD);
//Shuffleboard
GenericEntry 
    m_driveVelocity,
    m_steerVelocity, 
    m_steerAngle, 
    m_drivetrainvolts, 
    m_driveStaticFeedforward, 
    m_driveVelocityFeedforward, 
    m_steerStaticFeedforward,
    m_steerVelocityFeedforward,
    m_heading;
  ShuffleboardTab m_swerveModulesTab,m_drivetrainTab;

  public Map<Module,Double> m_driveVelFeedForwardSaver=new HashMap<Module,Double>();
  public Map<Module,Double> m_driveStaticFeedForwardSaver=new HashMap<Module,Double>();
  public Map<Module,Double> m_steerVelFeedForwardSaver=new HashMap<Module,Double>();
  public Map<Module,Double> m_steerStaticFeedForwardSaver=new HashMap<Module,Double>();
  // modules needed to distigue in chooser
  Module m_dummyModule = Module.create(ModuleConstants.NONE);
  Module m_allModule = Module.create(ModuleConstants.NONE);
  // previous module for switching
  Module m_prevModule = m_dummyModule;
  SendableChooser<Module> m_module = new SendableChooser<>();
 ;

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
    
    LiveWindow.disableAllTelemetry();
    m_swerveModulesTab.add("Module Chooser", m_module);
    m_drivetrainTab = Shuffleboard.getTab("Drive");
    m_swerveModulesTab = Shuffleboard.getTab("Swerve Modules");
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
   * Resets the pigeon yaw, but only if it hasn't already been reset. Will reset it to {@link Constants.drive.kStartingHeadingDegrees}
   */
  public void initializePigeonYaw(boolean force) {
    if (!m_hasResetYaw || force) {
      m_hasResetYaw = true;
      setPigeonYaw(Constants.drive.kStartingHeadingDegrees);
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
    SwerveDriveKinematics.desaturateWheelSpeeds(m_swerveModuleStates, Constants.drive.kMaxSpeed);
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
    SwerveDriveKinematics.desaturateWheelSpeeds(m_swerveModuleStates, Constants.drive.kMaxSpeed);
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
  private void setupDrivetrain() {
    // inputs
    m_heading = m_drivetrainTab.add("Set Heading (-pi to pi)", 0).getEntry();
    
    // add PID controlers
    m_drivetrainTab.add("xController", getXController());
    m_drivetrainTab.add("yController", getYController());
    m_drivetrainTab.add("rotationController", getRotationController());

    m_drivetrainTab.addNumber("getAngle", () -> getAngleHeading());
    m_drivetrainTab.addNumber("heading PID output", () -> m_headingPIDOutput);

    m_drivetrainTab.addNumber("Gyro X", () -> getAngularRate(0));
    m_drivetrainTab.addNumber("Gyro Y", () -> getAngularRate(1));
    m_drivetrainTab.addNumber("Gyro Z", () -> getAngularRate(2));

    m_drivetrainTab.add(getXController());
    m_drivetrainTab.add(getYController());
    m_drivetrainTab.add(getRotationController());
  }
  private void setupModules(){
    // inputs
    m_driveVelocity = m_swerveModulesTab.add("Set Drive Velocity", 0).getEntry();
    m_steerVelocity = m_swerveModulesTab.add("Set Steer Velocity", 0).getEntry();
    m_steerAngle = m_swerveModulesTab.add("Set Steer Angle", 0).getEntry();
    m_drivetrainvolts = m_swerveModulesTab.add("Set Volts", 0).getEntry();
    m_driveStaticFeedforward = m_swerveModulesTab.add("Drive kS FF", 0).getEntry();
    m_driveVelocityFeedforward = m_swerveModulesTab.add("Drive kV FF", 0).getEntry();
    m_steerStaticFeedforward = m_swerveModulesTab.add("Steer kS FF", 0).getEntry();
    m_steerVelocityFeedforward = m_swerveModulesTab.add("Steer kV k FF", 0).getEntry();
    
    // Desired Drive Velocitys
    // m_swerveModulesTab.addNumber("FL desired speed", () -> Robot.drive.swerveModuleStates[0].speedMetersPerSecond);
    // m_swerveModulesTab.addNumber("FR desired speed", () -> Robot.drive.swerveModuleStates[1].speedMetersPerSecond);
    // m_swerveModulesTab.addNumber("BL desired speed", () -> Robot.drive.swerveModuleStates[2].speedMetersPerSecond);
    // m_swerveModulesTab.addNumber("BR desired speed", () -> Robot.drive.swerveModuleStates[3].speedMetersPerSecond);

    // Drive PID output
    // m_swerveModulesTab.addNumber("FL PID Output", () -> Robot.drive.m_modules[0].getDrivePIDOutput());
    // m_swerveModulesTab.addNumber("FR PID Output", () -> Robot.drive.m_modules[1].getDrivePIDOutput());
    // m_swerveModulesTab.addNumber("BL PID Output", () -> Robot.drive.m_modules[2].getDrivePIDOutput());
    // m_swerveModulesTab.addNumber("BR PID Output", () -> Robot.drive.m_modules[3].getDrivePIDOutput());

    // get drive velocity
    // m_swerveModulesTab.addNumber("Vel FL Raw", () -> Robot.drive.m_modules[0].getDriveVelocity());
    // m_swerveModulesTab.addNumber("Vel FR Raw", () -> Robot.drive.m_modules[1].getDriveVelocity());
    // m_swerveModulesTab.addNumber("Vel BL Raw", () -> Robot.drive.m_modules[2].getDriveVelocity());
    // m_swerveModulesTab.addNumber("Vel BR Raw", () -> Robot.drive.m_modules[3].getDriveVelocity());

    // drivePIDS
    // m_swerveModulesTab.add("Drive PID FL", Robot.drive.m_modules[0].getDrivePID());
    // m_swerveModulesTab.add("Drive PID FR", Robot.drive.m_modules[1].getDrivePID());
    // m_swerveModulesTab.add("Drive PID BL", Robot.drive.m_modules[2].getDrivePID());
    // m_swerveModulesTab.add("Drive PID BR", Robot.drive.m_modules[3].getDrivePID());

    //Median Filltered Velocity Values
    // m_swerveModulesTab.addNumber("Vel FL Filtered", () -> Robot.drive.m_modules[0].getDriveVelocityFilltered());
    // m_swerveModulesTab.addNumber("Vel FR Filtered", () -> Robot.drive.m_modules[1].getDriveVelocityFilltered());
    // m_swerveModulesTab.addNumber("Vel BL Filtered", () -> Robot.drive.m_modules[2].getDriveVelocityFilltered());
    // m_swerveModulesTab.addNumber("Vel BR Filtered", () -> Robot.drive.m_modules[3].getDriveVelocityFilltered());

    // Desired Steer angles
    // m_swerveModulesTab.addNumber("FL desired angle", () -> Robot.drive.m_swerveModuleStates[0].angle.getDegrees());
    // m_swerveModulesTab.addNumber("FR desired angle", () -> Robot.drive.m_swerveModuleStates[1].angle.getDegrees());
    // m_swerveModulesTab.addNumber("BL desired angle", () -> Robot.drive.m_swerveModuleStates[2].angle.getDegrees());
    // m_swerveModulesTab.addNumber("BR desired angle", () -> Robot.drive.m_swerveModuleStates[3].angle.getDegrees());

    // Steer angles
    m_swerveModulesTab.addNumber("Angle FL", () -> m_modules[0].getSteerAngle());
    m_swerveModulesTab.addNumber("Angle FR", () -> m_modules[1].getSteerAngle());
    m_swerveModulesTab.addNumber("Angle BL", () -> m_modules[2].getSteerAngle());
    m_swerveModulesTab.addNumber("Angle BR", () -> m_modules[3].getSteerAngle());

    // Steer Velocity
    m_swerveModulesTab.addNumber("Steer Vel FL", () -> m_modules[0].getSteerVelocity());
    m_swerveModulesTab.addNumber("Steer Vel FR", () -> m_modules[1].getSteerVelocity());
    m_swerveModulesTab.addNumber("Steer Vel BL", () -> m_modules[2].getSteerVelocity());
    m_swerveModulesTab.addNumber("Steer Vel BR", () -> m_modules[3].getSteerVelocity());

    //Steer PID
    m_swerveModulesTab.add("Steer PID FL", m_modules[0].getSteerPID());
    m_swerveModulesTab.add("Steer PID FR", m_modules[1].getSteerPID());
    m_swerveModulesTab.add("Steer PID BL", m_modules[2].getSteerPID());
    m_swerveModulesTab.add("Steer PID BR", m_modules[3].getSteerPID());
  }
  private void setUpFeedforwardHashmap(){
    m_driveStaticFeedForwardSaver.put(m_dummyModule,0.0);
    m_driveVelFeedForwardSaver.put(m_allModule,Constants.drive.kDriveKVAll);
    m_driveStaticFeedForwardSaver.put(m_modules[0],TestDriveConstants.kDriveKSFrontLeft);
    m_driveStaticFeedForwardSaver.put(m_modules[1],TestDriveConstants.kDriveKSFrontRight);
    m_driveStaticFeedForwardSaver.put(m_modules[2],TestDriveConstants.kDriveKSBackLeft);
    m_driveStaticFeedForwardSaver.put(m_modules[3],TestDriveConstants.kDriveKSBackRight);
    
    m_driveVelFeedForwardSaver.put(m_dummyModule,0.0);
    m_driveStaticFeedForwardSaver.put(m_allModule,Constants.drive.kDriveKSAll);
    m_driveVelFeedForwardSaver.put(m_modules[0],TestDriveConstants.kDriveKVFrontLeft);
    m_driveVelFeedForwardSaver.put(m_modules[1],TestDriveConstants.kDriveKVFrontRight);
    m_driveVelFeedForwardSaver.put(m_modules[2],TestDriveConstants.kDriveKVBackLeft);
    m_driveVelFeedForwardSaver.put(m_modules[3],TestDriveConstants.kDriveKVBackRight);
    

    m_steerStaticFeedForwardSaver.put(m_dummyModule,0.0);
    m_steerVelFeedForwardSaver.put(m_allModule,Constants.drive.kDriveKVAll);
    m_steerStaticFeedForwardSaver.put(m_modules[0],TestDriveConstants.kDriveKSFrontLeft);
    m_steerStaticFeedForwardSaver.put(m_modules[1],TestDriveConstants.kDriveKSFrontRight);
    m_steerStaticFeedForwardSaver.put(m_modules[2],TestDriveConstants.kDriveKSBackLeft);
    m_steerStaticFeedForwardSaver.put(m_modules[3],TestDriveConstants.kDriveKSBackRight);
    
    m_steerVelFeedForwardSaver.put(m_dummyModule,0.0);
    m_steerStaticFeedForwardSaver.put(m_allModule,Constants.drive.kDriveKSAll);
    m_steerVelFeedForwardSaver.put(m_modules[0],TestDriveConstants.kDriveKVFrontLeft);
    m_steerVelFeedForwardSaver.put(m_modules[1],TestDriveConstants.kDriveKVFrontRight);
    m_steerVelFeedForwardSaver.put(m_modules[2],TestDriveConstants.kDriveKVBackLeft);
    m_steerVelFeedForwardSaver.put(m_modules[3],TestDriveConstants.kDriveKVBackRight);
  }
  public double getRequestedHeading() {
    return m_heading.getDouble(0);
  }
  public double getRequestedDriveVelocity() {
    return m_driveVelocity.getDouble(0);
  }
  public double getRequestedSteerVelocity() {
    return m_steerVelocity.getDouble(0);
  }
  public double getRequestedVolts(){
    return m_drivetrainvolts.getDouble(0);
  }
  public double getRequestedSteerAngle() {
    return m_steerAngle.getDouble(0);
  }
  public double getDriveStaticFeedforward() {
    return m_driveStaticFeedforward.getDouble(0);
  }
  public double getDriveVelocityFeedforward() {
    return m_driveVelocityFeedforward.getDouble(0);
  }
  public void setDriveModuleFeedforward(){
    //revert to previous saved feed forward data if changed
    
    if (m_prevModule != m_module.getSelected()){
      m_driveStaticFeedforward.setDouble(m_driveStaticFeedForwardSaver.get(m_module.getSelected()));
      m_driveVelocityFeedforward.setDouble(m_driveVelFeedForwardSaver.get(m_module.getSelected()));
      m_prevModule = m_module.getSelected();
    }
    
    // update saved feedforward data
    m_driveStaticFeedForwardSaver.replace(m_module.getSelected(),m_driveStaticFeedforward.getDouble(0) );
    m_driveVelFeedForwardSaver.replace(m_module.getSelected(),m_driveVelocityFeedforward.getDouble(0) );
    
    //to set all modules to same feedforward values if all
    if (m_module.getSelected() == m_allModule){
      for(int i = 0; i < 4; i++){
        m_modules[i].setDriveFeedForwardValues(m_driveStaticFeedForwardSaver.get(m_module.getSelected()), m_driveVelFeedForwardSaver.get(m_module.getSelected()));
      }
    }
    //set selected module
    m_module.getSelected().setDriveFeedForwardValues(m_driveStaticFeedForwardSaver.get(m_module.getSelected()),m_driveVelFeedForwardSaver.get(m_module.getSelected()));
  }

  public void setSteerModuleFeedforward(){
    //revert to previous saved feed forward data if changed
    
    if (m_prevModule != m_module.getSelected()){
      m_steerStaticFeedforward.setDouble(m_steerStaticFeedForwardSaver.get(m_module.getSelected()));
      m_steerVelocityFeedforward.setDouble(m_steerVelFeedForwardSaver.get(m_module.getSelected()));
      m_prevModule = m_module.getSelected();
    }
    
    // update saved feedforward data
    m_steerStaticFeedForwardSaver.replace(m_module.getSelected(),m_steerStaticFeedforward.getDouble(0) );
    m_steerVelFeedForwardSaver.replace(m_module.getSelected(),m_steerVelocityFeedforward.getDouble(0) );
    
    //to set all modules to same feedforward values if all
    if (m_module.getSelected() == m_allModule){
      for(int i = 0; i < 4; i++){
        m_modules[i].setDriveFeedForwardValues(m_steerStaticFeedForwardSaver.get(m_module.getSelected()), m_steerVelFeedForwardSaver.get(m_module.getSelected()));
      }
    }
    //set selected module
    m_module.getSelected().setDriveFeedForwardValues(m_steerStaticFeedForwardSaver.get(m_module.getSelected()),m_steerVelFeedForwardSaver.get(m_module.getSelected()));
  }

}