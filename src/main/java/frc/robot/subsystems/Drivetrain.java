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
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.ModuleConstants;
import frc.robot.util.LogManager;

/** 
 * Represents a swerve drive style drivetrain.
 * 
 * Module IDs are:
 * 1: Front left
 * 2: Front right
 * 3: Back left
 * 4: Back right
 */
public class Drivetrain extends SubsystemBase {

  // This is left intentionally public
  public final Module[] m_modules;
  
  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
    new Translation2d(DriveConstants.kTrackWidth / 2, DriveConstants.kTrackWidth / 2),
    new Translation2d(DriveConstants.kTrackWidth / 2, -DriveConstants.kTrackWidth / 2),
    new Translation2d(-DriveConstants.kTrackWidth / 2, DriveConstants.kTrackWidth / 2),
    new Translation2d(-DriveConstants.kTrackWidth / 2, -DriveConstants.kTrackWidth / 2)
  );

  // Pigeon
  private final WPI_Pigeon2 m_pigeon = new WPI_Pigeon2(DriveConstants.kPigeon, Constants.kCanivoreCAN);
  private boolean m_hasResetYaw = false; // the initial yaw has been set 

  private double m_headingPIDOutput = 0;

  // Odometry
  private final SwerveDriveOdometry m_odometry;
  private Pose2d m_robotPose = new Pose2d();

  // Displays the field with the robots estimated pose on it
  private final Field2d m_fieldDisplay = new Field2d();
  
  // PID Controllers
  private final PIDController m_xController = new PIDController(0.1, 0, 0);
  private final PIDController m_yController = new PIDController(0.1, 0, 0);
  private final PIDController m_rotationController = new PIDController(DriveConstants.kHeadingP, DriveConstants.kHeadingI, DriveConstants.kHeadingD);

  //Shuffleboard
  private GenericEntry 
    m_driveVelocity,
    m_steerVelocity, 
    m_steerAngle, 
    m_drivetrainVolts, 
    m_driveStaticFeedforward, 
    m_driveVelocityFeedforward, 
    m_steerStaticFeedforward,
    m_steerVelocityFeedforward,
    m_heading;
  private ShuffleboardTab m_swerveModulesTab, m_drivetrainTab;

  private Double[] m_driveVelFeedForwardSaver = new Double[4];
  private Double[] m_driveStaticFeedForwardSaver = new Double[4];
  private Double[] m_steerVelFeedForwardSaver = new Double[4];
  private Double[] m_steerStaticFeedForwardSaver = new Double[4];
  
  private SendableChooser<Module> m_moduleChooser = new SendableChooser<>();
  // modules needed to distinguish in chooser
  private Module m_prevModule;

  /**
   * Creates a new Swerve Style Drivetrain.
   * @param drivetrainTab the shuffleboard tab to display drivetrain data on
   * @param swerveModulesTab the shuffleboard tab to display module data on
   */
  public Drivetrain(ShuffleboardTab drivetrainTab, ShuffleboardTab swerveModulesTab) {

    LiveWindow.disableAllTelemetry();
    m_drivetrainTab = drivetrainTab;
    m_swerveModulesTab = swerveModulesTab;
    
    m_modules = new Module[] {
      Module.create(ModuleConstants.COMP_FL, m_swerveModulesTab),
      Module.create(ModuleConstants.COMP_FR, m_swerveModulesTab),
      Module.create(ModuleConstants.COMP_BL, m_swerveModulesTab),
      Module.create(ModuleConstants.COMP_BR, m_swerveModulesTab)
    };
    m_prevModule = m_modules[0];
    
    m_odometry = new SwerveDriveOdometry(m_kinematics, m_pigeon.getRotation2d(), getModulePositions(), m_robotPose);
    m_rotationController.enableContinuousInput(-Math.PI, Math.PI);
    DoubleSupplier[] poseSupplier = {() -> getPose().getX(), () -> getPose().getY(), () -> getPose().getRotation().getRadians()};
    LogManager.addDoubleArray("Pose2d", poseSupplier);
    
    m_fieldDisplay.setRobotPose(getPose());
    SmartDashboard.putData("Field", m_fieldDisplay);
  }

  @Override
  public void periodic() {
    updateDriveModuleFeedforwardShuffleboard();
    updateDriveModuleFeedforwardShuffleboard();

    updateOdometry();
    
    m_fieldDisplay.setRobotPose(getPose());
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
    setChassisSpeeds((
      fieldRelative
          ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_pigeon.getRotation2d())
          : new ChassisSpeeds(xSpeed, ySpeed, rot)
      )
    );
  }  

  /**
   * Drives the robot using the provided x speed, y speed, and  heading.
   * 
   * @param xSpeed speed of the robot in the x direction (forward)
   * @param ySpeed speed of the robot in the y direction (sideways)
   * @param heading target heading of the robot
   * @param fieldRelative whether the provided x and y speeds are relative to the field
   */
  public void driveHeading(double xSpeed, double ySpeed, double heading, boolean fieldRelative) {
    m_headingPIDOutput = m_rotationController.calculate(getAngleHeading(), heading);
    double rot = m_headingPIDOutput;
    setChassisSpeeds((
      fieldRelative
          ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_pigeon.getRotation2d())
          : new ChassisSpeeds(xSpeed, ySpeed, rot)
      )
    );
  }

  /**
   * Sets the chassis speeds of the robot.
   * 
   * @param chassisSpeeds the target chassis speeds
   */
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeed);
    setModuleStates(swerveModuleStates);
  }

  /**
   * Runs the PID controllers with the provided x, y, and rot values. Then, calls {@link #driveRot(double, double, double, boolean)} using the PID outputs.
   * 
   * @param xSpeed speed of the robot in the x direction (forward)
   * @param ySpeed speed of the robot in the y direction (sideways)
   * @param heading target heading of the robot
   */
  public void runChassisPID(double x, double y, double rot) {
    double xSpeed = m_xController.calculate(m_odometry.getPoseMeters().getX(), x);
    double ySpeed = m_yController.calculate(m_odometry.getPoseMeters().getY(), y);
    double rotRadians = m_rotationController.calculate(getAngleHeading(), rot);
    driveRot(xSpeed, ySpeed, rotRadians, true);
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
  public void resetOdometry(Pose2d pose, Rotation2d gyroAngle) {
    m_odometry.resetPosition(gyroAngle, getModulePositions(), pose);
  }
  
  /**
  * @return the pigeon's Rotation2d
  */
  public Rotation2d getRotation2d() {
    return m_pigeon.getRotation2d(); 
  }
  
  /**
  * Gets the angle heading from the pigeon.
  * 
  * @return the heading angle in radians, from -pi to pi
  */
  public double getAngleHeading() {
    return MathUtil.angleModulus(m_pigeon.getRotation2d().getRadians());
  }
  
  /**
  * Gets an array of all the swerve module positions.
  * 
  * @return an array of all swerve module positions
  */
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[] {
      m_modules[0].getPosition(),
      m_modules[1].getPosition(),
      m_modules[2].getPosition(),
      m_modules[3].getPosition()
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
  
  /**
   * Enables or disables the state deadband for all swerve modules.
   */
  public void enableStateDeadband(boolean stateDeadBand){
    for (int i = 0; i < 4; i++) {
      m_modules[i].enableStateDeadband(stateDeadBand);
    }
  }

  /**
   * Sets the optimize state for all swerve modules.
   */
  public void setAllOptimize(Boolean optimizeSate) {
    for (int i = 0; i < 4; i++) {
      m_modules[i].setOptimize(optimizeSate);
    }
  }
  
  /**
   * Stops all swerve modules.
   */
  public void stop() {
    for (int i = 0; i < 4; i++) {
      m_modules[i].stop();
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

  /**
   * Sets up the shuffleboard tab for the drivetrain.
   */
  public void setupDrivetrainShuffleboard() {
    // inputs
    m_heading = m_drivetrainTab.add("Set Heading (-pi to pi)", 0).getEntry();
    
    // add PID controllers
    m_drivetrainTab.add("xController", getXController());
    m_drivetrainTab.add("yController", getYController());
    m_drivetrainTab.add("rotationController", getRotationController());
    
    m_drivetrainTab.addNumber("getAngle", () -> getAngleHeading());
    m_drivetrainTab.addNumber("heading PID output", () -> m_headingPIDOutput);
    
    m_drivetrainTab.addNumber("Gyro X", () -> getAngularRate(0));
    m_drivetrainTab.addNumber("Gyro Y", () -> getAngularRate(1));
    m_drivetrainTab.addNumber("Gyro Z", () -> getAngularRate(2));
    
    // m_drivetrainTab.add("odometry", m_odometry);
    
    m_drivetrainTab.add(getXController());
    m_drivetrainTab.add(getYController());
    m_drivetrainTab.add(getRotationController());
  }

  /**
   * Sets up the shuffleboard tab for the swerve modules.
   */
  public void setupModulesShuffleboard() {
    setUpModuleChooser();
    setUpFeedforwardSavers();
    
    // inputs
    m_swerveModulesTab.add("Module Chooser", m_moduleChooser);
    m_driveVelocity = m_swerveModulesTab.add("Set Drive Velocity", 0).getEntry();
    m_steerVelocity = m_swerveModulesTab.add("Set Steer Velocity", 0).getEntry();
    m_steerAngle = m_swerveModulesTab.add("Set Steer Angle", 0).getEntry();
    m_drivetrainVolts = m_swerveModulesTab.add("Set Volts", 0).getEntry();
    m_driveStaticFeedforward = m_swerveModulesTab.add("Drive kS FF", m_driveStaticFeedForwardSaver[m_moduleChooser.getSelected().getModuleType().getID()]).getEntry();
    m_driveVelocityFeedforward = m_swerveModulesTab.add("Drive kV FF", m_driveVelFeedForwardSaver[m_moduleChooser.getSelected().getModuleType().getID()]).getEntry();
    m_steerStaticFeedforward = m_swerveModulesTab.add("Steer kS FF", m_steerStaticFeedForwardSaver[m_moduleChooser.getSelected().getModuleType().getID()]).getEntry();
    m_steerVelocityFeedforward = m_swerveModulesTab.add("Steer kV k FF", m_steerVelFeedForwardSaver[m_moduleChooser.getSelected().getModuleType().getID()]).getEntry();
    
    for (int i = 0; i < 4; i++) {
      m_modules[i].setupModulesShuffleboard();
    }
  }

  public GenericEntry getRequestedHeadingEntry() {
    return m_heading;
  }
  public GenericEntry getRequestedDriveVelocityEntry() {
    return m_driveVelocity;
  }
  public GenericEntry getRequestedSteerVelocityEntry() {
    return m_steerVelocity;
  }
  public GenericEntry getRequestedVoltsEntry() {
    return m_drivetrainVolts;
  }
  public GenericEntry getRequestedSteerAngleEntry() {
    return m_steerAngle;
  }
  public GenericEntry getDriveStaticFeedforwardEntry() {
    return m_driveStaticFeedforward;
  }
  public GenericEntry getDriveVelocityFeedforwardEntry() {
    return m_driveVelocityFeedforward;
  }
  public GenericEntry getSteerStaticFeedforwardEntry() {
    return m_steerStaticFeedforward;
  }
  public GenericEntry getSteerVelocityFeedforwardEntry() {
    return m_steerVelocityFeedforward;
  }

  /**
   * Updates the drive module feedforward values on shuffleboard.
   */
  public void updateDriveModuleFeedforwardShuffleboard() {
    // revert to previous saved feed forward data if changed
    if (m_prevModule != m_moduleChooser.getSelected()) {
      m_driveStaticFeedforward.setDouble(m_driveStaticFeedForwardSaver[m_moduleChooser.getSelected().getModuleType().getID()]);
      m_driveVelocityFeedforward.setDouble(m_driveVelFeedForwardSaver[m_moduleChooser.getSelected().getModuleType().getID()]);
      m_prevModule = m_moduleChooser.getSelected();
    }
    
    // update saved feedforward data
    m_driveStaticFeedForwardSaver[m_moduleChooser.getSelected().getModuleType().getID()] = m_driveStaticFeedforward.getDouble(0);
    m_driveVelFeedForwardSaver[m_moduleChooser.getSelected().getModuleType().getID()] = m_driveVelocityFeedforward.getDouble(0);
    
    // to set all modules to same feedforward values if all
    // if (m_module.getSelected() == m_allModule) {
    //   for(int i = 0; i < 4; i++) {
    //     m_modules[i].setDriveFeedForwardValues(m_driveStaticFeedForwardSaver.get(m_module.getSelected()), m_driveVelFeedForwardSaver.get(m_module.getSelected()));
    //   }
    // }
        
    //set selected module
    m_moduleChooser.getSelected().setDriveFeedForwardValues(m_driveStaticFeedForwardSaver[m_moduleChooser.getSelected().getModuleType().getID()],m_driveVelFeedForwardSaver[m_moduleChooser.getSelected().getModuleType().getID()]);
  }

  /**
   * Updates the steer module feedforward values on shuffleboard.
   */
  public void updateSteerModuleFeedforwardShuffleboard() {
    
    //revert to previous saved feed forward data if changed
    if (m_prevModule != m_moduleChooser.getSelected()) {
      m_steerStaticFeedforward.setDouble(m_steerStaticFeedForwardSaver[m_moduleChooser.getSelected().getModuleType().getID()]);
      m_steerVelocityFeedforward.setDouble(m_steerVelFeedForwardSaver[m_moduleChooser.getSelected().getModuleType().getID()]);
      m_prevModule = m_moduleChooser.getSelected();
    }
    
    // update saved feedforward data
    m_steerStaticFeedForwardSaver[m_moduleChooser.getSelected().getModuleType().getID()] = m_steerStaticFeedforward.getDouble(0);
    m_steerVelFeedForwardSaver[m_moduleChooser.getSelected().getModuleType().getID()] = m_steerVelocityFeedforward.getDouble(0);
    
    //to set all modules to same feedforward values if all
    // if (m_module.getSelected() == m_allModule) {
    //   for(int i = 0; i < 4; i++) {
    //     m_modules[i].setDriveFeedForwardValues(m_steerStaticFeedForwardSaver[m_module.getSelected().getModuleType().getID()], m_steerVelFeedForwardSaver[m_module.getSelected().getModuleType().getID()]);
    //   }
    // }
    
    //set selected module
    m_moduleChooser.getSelected().setDriveFeedForwardValues(m_steerStaticFeedForwardSaver[m_moduleChooser.getSelected().getModuleType().getID()],m_steerVelFeedForwardSaver[m_moduleChooser.getSelected().getModuleType().getID()]);
  }
  
  /**
   * Sets up feedforward savers.
   */
  private void setUpFeedforwardSavers() {
    m_driveStaticFeedForwardSaver = new Double[] {
      m_modules[0].getDriveFeedForwardKS(),
      m_modules[1].getDriveFeedForwardKS(),
      m_modules[2].getDriveFeedForwardKS(),
      m_modules[3].getDriveFeedForwardKS()
    };
    m_driveVelFeedForwardSaver = new Double[] {
      m_modules[0].getDriveFeedForwardKV(),
      m_modules[1].getDriveFeedForwardKV(),
      m_modules[2].getDriveFeedForwardKV(),
      m_modules[3].getDriveFeedForwardKV()
    };
    m_steerStaticFeedForwardSaver = new Double[] {
      m_modules[0].getSteerFeedForwardKS(),
      m_modules[1].getSteerFeedForwardKS(),
      m_modules[2].getSteerFeedForwardKS(),
      m_modules[3].getSteerFeedForwardKS()
    };
    m_steerVelFeedForwardSaver = new Double[] {
      m_modules[0].getSteerFeedForwardKV(),
      m_modules[1].getSteerFeedForwardKV(),
      m_modules[2].getSteerFeedForwardKV(),
      m_modules[3].getSteerFeedForwardKV()
    };
  }
  public Double[] getDriveStaticFeedforwardArray() {
    return m_driveStaticFeedForwardSaver;
  }
  public Double[] getDriveVelocityFeedforwardArray() {
    return m_driveVelFeedForwardSaver;
  }
  public Double[] getSteerStaticFeedforwardArray() {
    return m_steerStaticFeedForwardSaver;
  }
  public Double[] getSteerVelocityFeedforwardArray() {
    return m_steerVelFeedForwardSaver;
  }

  public SendableChooser<Module> getModuleChooser() {
    return m_moduleChooser;
  }

  /**
   * Sets up module chooser.
   */
  public void setUpModuleChooser() {
    m_moduleChooser.setDefaultOption("Front Left", m_modules[0]);
    m_moduleChooser.addOption("Front Right", m_modules[1]);
    m_moduleChooser.addOption("Back Left", m_modules[2]);
    m_moduleChooser.addOption("Back Right", m_modules[3]);
  }

}