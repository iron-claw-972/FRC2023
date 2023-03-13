package frc.robot.subsystems;

import java.util.ArrayList;

import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix.sensors.Pigeon2.AxisDirection;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.commands.test.CircleDrive;
import frc.robot.commands.test.DriveFeedForwardCharacterization;
import frc.robot.commands.test.SteerFeedForwardCharacterizationSingle;
import frc.robot.commands.test.TestDriveVelocity;
import frc.robot.commands.test.TestHeadingPID;
import frc.robot.commands.test.TestSteerAngle;
import frc.robot.constants.Constants;
import frc.robot.constants.VisionConstants;
import frc.robot.constants.swerve.DriveConstants;
import frc.robot.constants.swerve.ModuleConstants;
import frc.robot.util.LogManager;
import frc.robot.util.Vision;
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

  private ShuffleboardTab m_swerveModulesTab, m_drivetrainTab;

  // Odometry
  private final SwerveDrivePoseEstimator m_poseEstimator;

  // This is left intentionally public
  public final Module[] m_modules;

  private final WPI_Pigeon2 m_pigeon;
  private Vision m_vision;

  // PID Controllers for chassis movement
  private final PIDController m_xController;
  private final PIDController m_yController;
  private final PIDController m_rotationController;

  private boolean m_chargeStationVision = false;

  // Displays the field with the robots estimated pose on it
  private final Field2d m_fieldDisplay;

  private final PIDController m_pathplannerXController;
  private final PIDController m_pathplannerYController;
  private final PIDController m_pathplannerRotationController;

  //Shuffleboard
  private GenericEntry 
    m_driveVelocityEntry,
    m_steerVelocityEntry, 
    m_steerAngleEntry,
    m_driveStaticFeedforwardEntry, 
    m_driveVelocityFeedforwardEntry, 
    m_steerStaticFeedforwardEntry,
    m_steerVelocityFeedforwardEntry,
    m_xPosEntry,
    m_yPosEntry,
    m_headingEntry;
  
  private Double[] m_driveVelFeedForwardSaver = new Double[4];
  private Double[] m_driveStaticFeedForwardSaver = new Double[4];
  private Double[] m_steerVelFeedForwardSaver = new Double[4];
  private Double[] m_steerStaticFeedForwardSaver = new Double[4];
  
  private SendableChooser<Module> m_moduleChooser = new SendableChooser<>();
  // modules needed to distinguish in chooser
  private Module m_prevModule;

  boolean m_visionEnabled = true;

  int m_loggerStep = 0;

  /**
   * Creates a new Swerve Style Drivetrain.
   * @param drivetrainTab the shuffleboard tab to display drivetrain data on
   * @param swerveModulesTab the shuffleboard tab to display module data on
   */
  public Drivetrain(ShuffleboardTab drivetrainTab, ShuffleboardTab swerveModulesTab, Vision vision) {

    m_drivetrainTab = drivetrainTab;
    m_swerveModulesTab = swerveModulesTab;

    m_vision = vision;
    
    m_pigeon = new WPI_Pigeon2(DriveConstants.kPigeon, DriveConstants.kPigeonCAN);
    m_pigeon.configFactoryDefault();
    // Our pigeon is mounted with y forward, and z upward
    m_pigeon.configMountPose(AxisDirection.PositiveY, AxisDirection.PositiveZ);

    if (RobotBase.isReal()) {
      m_modules = new Module[] {
        new Module(ModuleConstants.FRONT_LEFT, swerveModulesTab),
        new Module(ModuleConstants.FRONT_RIGHT, swerveModulesTab),
        new Module(ModuleConstants.BACK_LEFT, swerveModulesTab),
        new Module(ModuleConstants.BACK_RIGHT, swerveModulesTab),
      };
    } else {
      m_modules = new ModuleSim[] {
        new ModuleSim(ModuleConstants.FRONT_LEFT, swerveModulesTab),
        new ModuleSim(ModuleConstants.FRONT_RIGHT, swerveModulesTab),
        new ModuleSim(ModuleConstants.BACK_LEFT, swerveModulesTab),
        new ModuleSim(ModuleConstants.BACK_RIGHT, swerveModulesTab),
      };
    }

    m_prevModule = m_modules[0];

    /*
     * By pausing init for a second before setting module offsets, we avoid a bug
     * with inverting motors.
     * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
     */
    Timer.delay(1.0);
    resetModulesToAbsolute();

    m_poseEstimator = new SwerveDrivePoseEstimator(
      DriveConstants.kKinematics,
      getYaw(),
      getModulePositions(),
      new Pose2d() // initial Odometry Location
    );
    m_poseEstimator.setVisionMeasurementStdDevs(VisionConstants.kBaseVisionPoseStdDevs);

    setYaw(DriveConstants.kStartingHeadingDegrees);

    m_xController = new PIDController(DriveConstants.kTranslationalP, 0, DriveConstants.kTranslationalD);
    m_yController = new PIDController(DriveConstants.kTranslationalP, 0, DriveConstants.kTranslationalD);
    m_rotationController = new PIDController(DriveConstants.kHeadingP, 0, DriveConstants.kHeadingD);
    m_rotationController.enableContinuousInput(-Math.PI, Math.PI);
    
    m_pathplannerXController = new PIDController(DriveConstants.kPathplannerTranslationalP, 0, DriveConstants.kPathplannerTranslationalD);
    m_pathplannerYController = new PIDController(DriveConstants.kPathplannerTranslationalP, 0, DriveConstants.kPathplannerTranslationalD);
    m_pathplannerRotationController = new PIDController(DriveConstants.kPathplannerHeadingP, 0, DriveConstants.kPathplannerHeadingD);
    m_pathplannerRotationController.enableContinuousInput(-Math.PI, Math.PI);

    m_fieldDisplay = new Field2d();
    m_fieldDisplay.setRobotPose(getPose());

    setupDrivetrainShuffleboard();
    setupModulesShuffleboard();
  }

  @Override
  public void periodic() {
    updateDriveModuleFeedforwardShuffleboard();
    updateDriveModuleFeedforwardShuffleboard();

    updateOdometry();
    
    m_fieldDisplay.setRobotPose(getPose());

    if (Constants.kLogging) updateLogs();
  }

  // PIDs for Chassis movement
  public PIDController getXController() { return m_xController; }
  public PIDController getYController() { return m_yController; }
  public PIDController getRotationController() { return m_rotationController; }

  // PIDs for Pathplanner
  public PIDController getPathplannerXController() { return m_pathplannerXController; }
  public PIDController getPathplannerYController() { return m_pathplannerYController; }
  public PIDController getPathplannerRotationController() { return m_pathplannerRotationController; }

  /**
   * @return chassis speed of swerve drive
   */
  public ChassisSpeeds getChassisSpeeds() {
    return DriveConstants.kKinematics.toChassisSpeeds(getModuleStates());
  }

  public ChassisSpeeds getFieldRelativeChassisSpeeds() {
    return ChassisSpeeds.fromFieldRelativeSpeeds(
      getChassisSpeeds(),
      getPose().getRotation()
    );
  }

  public double getChassisSpeedsMagnitude() {
    return Math.hypot(
      getFieldRelativeChassisSpeeds().vxMetersPerSecond,
      getFieldRelativeChassisSpeeds().vyMetersPerSecond
    );
  }

  public Rotation2d getFieldRelativeHeading() {
    return Rotation2d.fromRadians(Math.atan2(
      getFieldRelativeChassisSpeeds().vxMetersPerSecond,
      getFieldRelativeChassisSpeeds().vyMetersPerSecond
    ));
  }

  public Rotation2d getPitch() {
    return Rotation2d.fromDegrees(m_pigeon.getPitch());
  }
  
  public Rotation2d getRoll() {
    return Rotation2d.fromDegrees(m_pigeon.getRoll());
  }  
  
  /**
  * @return the pigeon's heading in a Rotation2d
  */
  public Rotation2d getYaw() {
    return (DriveConstants.kInvertGyro) ? Rotation2d.fromDegrees(MathUtil.inputModulus(180 - m_pigeon.getYaw(), -180, 180))
        : Rotation2d.fromDegrees(MathUtil.inputModulus(m_pigeon.getYaw(), -180, 180));
  }

  /**
  * Method to drive the robot using joystick info.
  *
  * @param xSpeed speed of the robot in the x direction (forward) in m/s
  * @param ySpeed speed of the robot in the y direction (sideways) in m/s
  * @param rot angular rate of the robot in rad/s
  * @param fieldRelative whether the provided x and y speeds are relative to the field
  * @param isOpenLoop whether to use velocity control for the drive motors
  */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean isOpenLoop) {           
    setChassisSpeeds((
      fieldRelative
          ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getYaw())
          : new ChassisSpeeds(xSpeed, ySpeed, rot)
      ),
      isOpenLoop
    );
  }

  /**
  * Sets the desired states for all swerve modules.
  * 
  * @param swerveModuleStates an array of module states to set swerve modules to. Order of the array matters here!
  */
  public void setModuleStates(SwerveModuleState[] swerveModuleStates, boolean isOpenLoop) {
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeed);
    for (int i = 0; i < 4; i++) {
      m_modules[i].setDesiredState(swerveModuleStates[i], isOpenLoop);
    }
  }
  
  /**
  * Sets the desired states for all swerve modules. Runs closed loop control. USe this function for pathplanner.
  * 
  * @param swerveModuleStates an array of module states to set swerve modules to. Order of the array matters here!
  */
  public void setModuleStates(SwerveModuleState[] swerveModuleStates) {
    setModuleStates(swerveModuleStates, false);
  }

  /**
  * Gets the current robot pose from the pose estimator.
  */
  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  /**
  * Resets the odometry to the given pose.
  * @param pose the pose to reset to.
  */
  public void resetOdometry(Pose2d pose) {
    m_poseEstimator.resetPosition(getYaw(), getModulePositions(), pose);
  }
  
  /**
   * Enables or disables the state deadband for all swerve modules. 
   * The state deadband determines if the robot will stop drive and steer motors when inputted drive velocity is low. 
   * It should be enabled for all regular driving, to prevent releasing the controls from setting the angles.
   */
  public void enableStateDeadband(boolean stateDeadBand){
    for (int i = 0; i < 4; i++) {
      m_modules[i].enableStateDeadband(stateDeadBand);
    }
  }

  PIDController m_balancePID = new PIDController(DriveConstants.kBalanceP, DriveConstants.kBalanceI, DriveConstants.kBalanceD);
  public PIDController getBalanceController() {
    return m_balancePID;
  }

  /**
  * Gets an array of SwerveModulePositions, which store the distance travleled by the drive and the steer angle.
  * 
  * @return an array of all swerve module positions
  */
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (Module mod : m_modules) {
      positions[mod.getModuleIndex()] = mod.getPosition();
    }
    return positions;
  }

  /**
   * Gets an array of SwerveModuleStates, which store the drive velocity and steer angle
   * @return an array of all swerve module positions
   */
  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (Module mod : m_modules) {
      states[mod.getModuleIndex()] = mod.getState();
    }
    return states;
  }

  /**
   * Sets the chassis speeds of the robot.
   * 
   * @param chassisSpeeds the target chassis speeds
   * @param isOpenLoop if open loop control should be used for the drive velocity
   */
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds, boolean isOpenLoop) {
    if (Robot.isSimulation()) {
      m_pigeon.getSimCollection().addHeading(
      + Units.radiansToDegrees(chassisSpeeds.omegaRadiansPerSecond * Constants.kLoopTime));
    }
    SwerveModuleState[] swerveModuleStates = DriveConstants.kKinematics.toSwerveModuleStates(chassisSpeeds);
    setModuleStates(swerveModuleStates, isOpenLoop);
  }

  /**
   * 
   * Resets the pigeon IMU's yaw.
   * 
   * @param degrees the new yaw angle, in degrees.
   */
  public void setYaw(double degrees) {
    // the odometry stores an offset from the current pigeon angle
    // changing the angle makes that offset inaccurate, so must reset the pose as well.
    // keep the same translation, but set the odometry angle to what we want the angle to be.
    resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(degrees)));
  }

  /**
   * 
   * Resets the pigeon IMU's yaw to the trajectory's intial state, flipped for alliance.
   * 
   * @param traj the trajectory to reset to.
   */
  public void setPigeonYaw(PathPlannerTrajectory traj) {
    traj = PathPlannerTrajectory.transformTrajectoryForAlliance(traj, DriverStation.getAlliance());
    setYaw(traj.getInitialHolonomicPose().getRotation().getDegrees());
  }

  public void resetModulesToAbsolute() {
    for (Module mod : m_modules) {
      mod.resetToAbsolute();
    }
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    // Updates pose based on encoders and gyro
    m_poseEstimator.update(getYaw(), getModulePositions());

    // Updates pose based on vision
    if (m_visionEnabled) {

      // The angle should be greater than 5 degrees if it goes over the charge station
      if (Math.abs(getPitch().getDegrees()) > 5 || Math.abs(getRoll().getDegrees()) > 5) {
        m_chargeStationVision = true;
      }

      // An array list of poses returned by different cameras
      ArrayList<EstimatedRobotPose> estimatedPoses = m_vision.getEstimatedPoses(m_poseEstimator.getEstimatedPosition());
      // The current position as a translation
      Translation2d currentEstimatedPoseTranslation = m_poseEstimator.getEstimatedPosition().getTranslation();
      for (int i = 0; i < estimatedPoses.size(); i++) {
        EstimatedRobotPose estimatedPose = estimatedPoses.get(i);
        // The position of the closest april tag as a translation
        Translation2d closestTagPoseTranslation = new Translation2d();
        for (int j = 0; j < estimatedPose.targetsUsed.size(); j++) {
          // The position of the current april tag
          Pose3d currentTagPose = m_vision.getTagPose(estimatedPose.targetsUsed.get(j).getFiducialId());
          // If it can't find the april tag's pose, don't run the rest of the for loop for this tag
          if (currentTagPose == null) {
            continue;
          }
          Translation2d currentTagPoseTranslation = currentTagPose.toPose2d().getTranslation();
          
          // If the current april tag position is closer than the closest one, this makes makes it the closest
          if (j == 0 || currentEstimatedPoseTranslation.getDistance(currentTagPoseTranslation) < currentEstimatedPoseTranslation.getDistance(closestTagPoseTranslation)) {
            closestTagPoseTranslation = currentTagPoseTranslation;
          }
        }

        // Adds the vision measurement for this camera
        m_poseEstimator.addVisionMeasurement(
          estimatedPose.estimatedPose.toPose2d(),
          estimatedPose.timestampSeconds,
          m_chargeStationVision ? VisionConstants.kChargeStationVisionPoseStdDevs.plus(
            currentEstimatedPoseTranslation.getDistance(closestTagPoseTranslation) * VisionConstants.kVisionPoseStdDevFactor
          ) : VisionConstants.kBaseVisionPoseStdDevs
        );
      }
      
      // If it used vision after going over the charge station, it should trust vision normally again
      if (estimatedPoses.size()>0) {
        m_chargeStationVision = false;
      }
      m_fieldDisplay.setRobotPose(m_poseEstimator.getEstimatedPosition());
    }
  }

  /**
   * Drives the robot using the provided x speed, y speed, and positional heading.
   * 
   * @param xSpeed speed of the robot in the x direction (forward)
   * @param ySpeed speed of the robot in the y direction (sideways)
   * @param heading target heading of the robot in radians
   * @param fieldRelative whether the provided x and y speeds are relative to the field
   */
  public void driveHeading(double xSpeed, double ySpeed, double heading, boolean fieldRelative) {
    double rot = m_rotationController.calculate(getYaw().getRadians(), heading);
    //SmartDashboard.putNumber("Heading PID Output", rot);
    setChassisSpeeds((
      fieldRelative
          ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_pigeon.getRotation2d())
          : new ChassisSpeeds(xSpeed, ySpeed, rot)
      ),
      false
    );
  }

  /**
   * Runs the PID controllers with the provided x, y, and rot values. Then, calls {@link #drive()} using the PID outputs.
   * This is based on the odometry of the chassis.
   * 
   * @param x the position to move to in the x, in meters
   * @param y the position to move to in the y, in meters
   * @param rot the angle to move to, in radians
   */
  public void runChassisPID(double x, double y, double rot) {
    double xSpeed = m_xController.calculate(m_poseEstimator.getEstimatedPosition().getX(), x);
    double ySpeed = m_yController.calculate(m_poseEstimator.getEstimatedPosition().getY(), y);
    double rotRadians = m_rotationController.calculate(getYaw().getRadians(), rot);
    drive(xSpeed, ySpeed, rotRadians, true, false);
  }
  
  /**
  * Returns the angular rate from the pigeon.
  * @param id 0 for x, 1 for y, 2 for z
  * @return the rate in rads/s from the pigeon
  */
  public double getAngularRate(int id) {

    // uses pass by reference and edits reference to array
    double[] rawGyros = new double[3];
    m_pigeon.getRawGyro(rawGyros);

    // outputs in deg/s, so convert to rad/s
    return Units.degreesToRadians(rawGyros[id]);
  }


  public void enableVision(boolean enabled) {
    m_visionEnabled = enabled;
  }

  /**
   * Sets the optimize state for all swerve modules.
   * Optimizing the state means the modules will not turn the steer motors more than 90 degrees for any one movement.
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


  // BELOW IS TELEMETRY STUFF

  /**
   * Sets up the shuffleboard tab for the drivetrain.
   */
  private void setupDrivetrainShuffleboard() {
    if (!Constants.kUseTelemetry) return;

    m_drivetrainTab.add("Field", m_fieldDisplay);
    SmartDashboard.putData("Field Display", m_fieldDisplay);

    m_drivetrainTab.add("Balance PID", m_balancePID);

    // inputs
    m_headingEntry = m_drivetrainTab.add("Set Heading (-pi to pi)", 0).getEntry();
    m_xPosEntry = m_drivetrainTab.add("Input X pos(m)",0).getEntry();
    m_yPosEntry = m_drivetrainTab.add("Input Y pos(m)",0).getEntry();
    
    // add PID controllers
    m_drivetrainTab.add("xController", getXController());
    m_drivetrainTab.add("yController", getYController());
    m_drivetrainTab.add("rotationController", getRotationController());
    
    m_drivetrainTab.add("PP xController", getPathplannerXController());
    m_drivetrainTab.add("PP yController", getPathplannerYController());
    m_drivetrainTab.add("PP rotationController", getPathplannerRotationController());
    
    // add angles
    m_drivetrainTab.addNumber("Yaw (deg)", () -> getYaw().getDegrees());
    m_drivetrainTab.addNumber("estimated X", () -> m_poseEstimator.getEstimatedPosition().getX());
    m_drivetrainTab.addNumber("estimated Y", () -> m_poseEstimator.getEstimatedPosition().getY());
    m_drivetrainTab.addNumber("getPitch", () -> m_pigeon.getPitch());
    m_drivetrainTab.addNumber("getRoll", () -> m_pigeon.getRoll());
    m_drivetrainTab.addNumber("getYaw", () -> m_pigeon.getYaw());
    
    m_drivetrainTab.addNumber("Gyro X", () -> getAngularRate(0));
    m_drivetrainTab.addNumber("Gyro Y", () -> getAngularRate(1));
    m_drivetrainTab.addNumber("Gyro Z", () -> getAngularRate(2));

    m_drivetrainTab.addNumber("Chassis Velocity", () -> getChassisSpeedsMagnitude());
  }

  /**
   * Sets up the shuffleboard tab for the swerve modules.
   */
  private void setupModulesShuffleboard() {
    if (Constants.kUseTelemetry) {
      
      m_moduleChooser.setDefaultOption("Front Left", m_modules[0]);
      m_moduleChooser.addOption("Front Right", m_modules[1]);
      m_moduleChooser.addOption("Back Left", m_modules[2]);
      m_moduleChooser.addOption("Back Right", m_modules[3]);

      setUpFeedforwardSavers();
      
      // inputs
      m_swerveModulesTab.add("Module Chooser", m_moduleChooser);
      m_driveVelocityEntry = m_swerveModulesTab.add("Set Drive Velocity", 0).getEntry();
      m_steerVelocityEntry = m_swerveModulesTab.add("Set Steer Velocity", 0).getEntry();
      m_steerAngleEntry = m_swerveModulesTab.add("Set Steer Angle", 0).getEntry();
      m_driveStaticFeedforwardEntry = m_swerveModulesTab.add(
        "Drive kS FF", 
        m_driveStaticFeedForwardSaver[m_moduleChooser.getSelected().getModuleIndex()]
      ).getEntry();

      m_driveVelocityFeedforwardEntry = m_swerveModulesTab.add(
        "Drive kV FF", 
        m_driveVelFeedForwardSaver[m_moduleChooser.getSelected().getModuleIndex()]
      ).getEntry();

      m_steerStaticFeedforwardEntry = m_swerveModulesTab.add(
        "Steer kS FF", 
        m_steerStaticFeedForwardSaver[m_moduleChooser.getSelected().getModuleIndex()]
      ).getEntry();

      m_steerVelocityFeedforwardEntry = m_swerveModulesTab.add(
        "Steer kV FF", 
        m_steerVelFeedForwardSaver[m_moduleChooser.getSelected().getModuleIndex()]
      ).getEntry();
    }
  }

  public double getRequestedHeading(double defaultValue) {
    if (!Constants.kUseTelemetry) return defaultValue;
    return m_headingEntry.getDouble(defaultValue);
  }
  public double getRequestedDriveVelocity(double defaultValue) {
    if (!Constants.kUseTelemetry) return defaultValue;
    return m_driveVelocityEntry.getDouble(defaultValue);
  }
  public double getRequestedSteerVelocity(double defaultValue) {
    if (!Constants.kUseTelemetry) return defaultValue;
    return m_steerVelocityEntry.getDouble(defaultValue);
  }
  public double getRequestedSteerAngle(double defaultValue) {
    if (!Constants.kUseTelemetry) return defaultValue;
    return m_steerAngleEntry.getDouble(defaultValue);
  }
  public double getRequestedXPos(double defaultValue) {
    if (!Constants.kUseTelemetry) return defaultValue;
    return m_xPosEntry.getDouble(defaultValue);
  }
  public double getRequestedYPos(double defaultValue) {
    if (!Constants.kUseTelemetry) return defaultValue;
    return m_yPosEntry.getDouble(defaultValue);
  }

  public void setDriveVelocityFeedforwardEntry(double value) {
    if (!Constants.kUseTelemetry) return;
    m_driveVelocityFeedforwardEntry.setDouble(value);
  }
  public void setDriveStaticFeedforwardEntry(double value) {
    if (!Constants.kUseTelemetry) return;
    m_driveStaticFeedforwardEntry.setDouble(value);
  }
  public void setSteerStaticFeedforwardEntry(double value) {
    if (!Constants.kUseTelemetry) return;
    m_steerStaticFeedforwardEntry.setDouble(value);
  }
  public void setSteerVelocityFeedforwardEntry(double value) {
    if (!Constants.kUseTelemetry) return;
    m_steerVelocityFeedforwardEntry.setDouble(value);
  }

  /**
   * Updates the drive module feedforward values on shuffleboard.
   */
  public void updateDriveModuleFeedforwardShuffleboard() {
    if (!Constants.kUseTelemetry) return;
    // revert to previous saved feed forward data if changed
    if (m_prevModule != m_moduleChooser.getSelected()) {
      m_driveStaticFeedforwardEntry.setDouble(
        m_driveStaticFeedForwardSaver[m_moduleChooser.getSelected().getModuleIndex()]
      );
      m_driveVelocityFeedforwardEntry.setDouble(
        m_driveVelFeedForwardSaver[m_moduleChooser.getSelected().getModuleIndex()]
      );
      m_prevModule = m_moduleChooser.getSelected();
    }
    
    // update saved feedforward data
    m_driveStaticFeedForwardSaver[m_moduleChooser.getSelected().getModuleIndex()] = 
      m_driveStaticFeedforwardEntry.getDouble(0);
    m_driveVelFeedForwardSaver[m_moduleChooser.getSelected().getModuleIndex()] = 
      m_driveVelocityFeedforwardEntry.getDouble(0);
    
    // to set all modules to same feedforward values if all
    // if (m_module.getSelected() == m_allModule) {
    //   for(int i = 0; i < 4; i++) {
    //     m_modules[i].setDriveFeedForwardValues(m_driveStaticFeedForwardSaver.get(m_module.getSelected()), m_driveVelFeedForwardSaver.get(m_module.getSelected()));
    //   }
    // }
        
    //set selected module
    m_moduleChooser.getSelected().setDriveFeedForwardValues(
      m_driveStaticFeedForwardSaver[m_moduleChooser.getSelected().getModuleIndex()],
      m_driveVelFeedForwardSaver[m_moduleChooser.getSelected().getModuleIndex()]
    );
  }

  /**
   * Updates the steer module feedforward values on shuffleboard.
   */
  public void updateSteerModuleFeedforwardShuffleboard() {
    if (!Constants.kUseTelemetry) return;
    
    //revert to previous saved feed forward data if changed
    if (m_prevModule != m_moduleChooser.getSelected()) {
      m_steerStaticFeedforwardEntry.setDouble(
        m_steerStaticFeedForwardSaver[m_moduleChooser.getSelected().getModuleIndex()]
      );
      m_steerVelocityFeedforwardEntry.setDouble(
        m_steerVelFeedForwardSaver[m_moduleChooser.getSelected().getModuleIndex()]
      );
      m_prevModule = m_moduleChooser.getSelected();
    }
    
    // update saved feedforward data
    m_steerStaticFeedForwardSaver[m_moduleChooser.getSelected().getModuleIndex()] = 
      m_steerStaticFeedforwardEntry.getDouble(0);
    m_steerVelFeedForwardSaver[m_moduleChooser.getSelected().getModuleIndex()] = 
      m_steerVelocityFeedforwardEntry.getDouble(0);
    
    //to set all modules to same feedforward values if all
    // if (m_module.getSelected() == m_allModule) {
    //   for(int i = 0; i < 4; i++) {
    //     m_modules[i].setDriveFeedForwardValues(m_steerStaticFeedForwardSaver[m_module.getSelected().getId()], m_steerVelFeedForwardSaver[m_module.getSelected().getId()]);
    //   }
    // }
    
    //set selected module
    m_moduleChooser.getSelected().setDriveFeedForwardValues(
      m_steerStaticFeedForwardSaver[m_moduleChooser.getSelected().getModuleIndex()],
      m_steerVelFeedForwardSaver[m_moduleChooser.getSelected().getModuleIndex()]
    );
  }
  
  public Module getModuleChoosen() {
    if (!Constants.kUseTelemetry) return m_modules[0];
    return m_moduleChooser.getSelected();
  }

   /**
   * Adds the test commands to shuffleboard so they can be run that way.
   */
  public void addTestCommands(ShuffleboardTab testTab, GenericEntry testEntry)  {
    if (Constants.kUseTelemetry) {
      testTab.add("Circle Drive", new CircleDrive(this));
      testTab.add("Drive FeedForward", new DriveFeedForwardCharacterization(this));
      testTab.add("Steer Single FeedForward", new SteerFeedForwardCharacterizationSingle(this));
      testTab.add("Test Drive Velocity", new TestDriveVelocity(this, testEntry));
      testTab.add("Heading PID", new TestHeadingPID(this, testEntry));
      testTab.add("Steer angle", new TestSteerAngle(this, testEntry));
      testTab.add("Reset Pose", new InstantCommand(()-> {
        this.resetOdometry(
          new Pose2d(
            this.getRequestedXPos(0),
            this.getRequestedYPos(0), 
            new Rotation2d(this.getRequestedHeading(0))
          ));
        }
      ));
    }
  }

  public void updateLogs() {

    m_loggerStep++;
    if (m_loggerStep < 4) return;
    m_loggerStep = 0;

    double[] pose = {
      getPose().getX(),
      getPose().getY(),
      getPose().getRotation().getRadians()
    };
    LogManager.addDoubleArray("Swerve/Pose2d", pose);

    double[] actualStates = {
      m_modules[0].getAngle().getRadians(),
      m_modules[0].getState().speedMetersPerSecond,
      m_modules[1].getAngle().getRadians(),
      m_modules[1].getState().speedMetersPerSecond,
      m_modules[2].getAngle().getRadians(),
      m_modules[2].getState().speedMetersPerSecond,
      m_modules[3].getAngle().getRadians(),
      m_modules[3].getState().speedMetersPerSecond
    };
    LogManager.addDoubleArray("Swerve/actual swerve states", actualStates);

    double[] desiredStates = {
      m_modules[0].getDesiredAngle().getRadians(),
      m_modules[0].getDesiredVelocity(),
      m_modules[1].getDesiredAngle().getRadians(),
      m_modules[1].getDesiredVelocity(),
      m_modules[2].getDesiredAngle().getRadians(),
      m_modules[2].getDesiredVelocity(),
      m_modules[3].getDesiredAngle().getRadians(),
      m_modules[3].getDesiredVelocity()
    };
    LogManager.addDoubleArray("Swerve/desired swerve states", desiredStates);
  }
}