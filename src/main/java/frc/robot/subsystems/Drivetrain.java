package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.util.PracticeModeType;

/** Represents a swerve drive style drivetrain.
 * Module IDs are:
 * 1: Front left
 * 2: Front right
 * 3: Back left
 * 4: Back right
*/
public class Drivetrain extends SubsystemBase {

  // public final Translation2d[] m_moduleLocations = new Translation2d[]{
    // new Translation2d(Constants.drive.kTrackWidth / 2, Constants.drive.kTrackWidth / 2),
    // new Translation2d(Constants.drive.kTrackWidth / 2, -Constants.drive.kTrackWidth / 2),
    // new Translation2d(-Constants.drive.kTrackWidth / 2, Constants.drive.kTrackWidth / 2),
    // new Translation2d(-Constants.drive.kTrackWidth / 2, -Constants.drive.kTrackWidth / 2)
  // };
  // public final SwerveModulePosition[] m_swerveModulePositions = new SwerveModulePosition[4];
  public SwerveModuleState[] m_swerveModuleStates = new SwerveModuleState[] {
    new SwerveModuleState(),
    new SwerveModuleState(),
    new SwerveModuleState(),
    new SwerveModuleState()
  };

  public final SwerveModule[] m_modules = new SwerveModule[]{
    new SwerveModule(
      Constants.drive.kDriveFrontLeft,
      Constants.drive.kSteerFrontLeft,
      Constants.drive.kEncoderFrontLeft,
      Constants.drive.kSteerOffsetFrontLeft
    ),
    new SwerveModule(
      Constants.drive.kDriveFrontRight,
      Constants.drive.kSteerFrontRight,
      Constants.drive.kEncoderFrontRight,
      Constants.drive.kSteerOffsetFrontRight
    ),
    new SwerveModule(
      Constants.drive.kDriveBackLeft,
      Constants.drive.kSteerBackLeft,
      Constants.drive.kEncoderBackLeft,
      Constants.drive.kSteerOffsetBackLeft
    ),
    new SwerveModule(
      Constants.drive.kDriveBackRight,
      Constants.drive.kSteerBackRight,
      Constants.drive.kEncoderBackRight,
      Constants.drive.kSteerOffsetBackRight
    )
  };

  public final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
    new Translation2d(Constants.drive.kTrackWidth / 2, Constants.drive.kTrackWidth / 2),
    new Translation2d(Constants.drive.kTrackWidth / 2, -Constants.drive.kTrackWidth / 2),
    new Translation2d(-Constants.drive.kTrackWidth / 2, Constants.drive.kTrackWidth / 2),
    new Translation2d(-Constants.drive.kTrackWidth / 2, -Constants.drive.kTrackWidth / 2)
  );

  private final WPI_Pigeon2 m_pigeon = new WPI_Pigeon2(Constants.drive.kPigeon, Constants.kCanivoreCAN);
  private boolean m_hasResetYaw = false;

  public double headingPIDOutput = 0;


  private final SwerveDriveOdometry m_odometry;

  public boolean isSlewDrive = false;
  
  private PIDController xController = new PIDController(0, 0, 0);
  private PIDController yController = new PIDController(0, 0, 0);
  private PIDController rotationController = new PIDController(0.1, 0, 0);

  public Drivetrain() {
    m_odometry = new SwerveDriveOdometry(m_kinematics, m_pigeon.getRotation2d(), getModulePositions());
  }

  @Override
  public void periodic() {
    updateOdometry();
  }

  public void setPigeonYaw(double degrees) {
    m_pigeon.setYaw(degrees);
  }

  /**
   * Resets the pigeon yaw, but only if it hasn't already been reset. Will reset it to {@link Constants.drive.kStartingHeadingDegrees}
   */
  public void initializePigeonYaw() {
    if (!m_hasResetYaw) {
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
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {

    if (Robot.shuffleboard.getPracticeModeType() == PracticeModeType.TUNE_HEADING_PID) {
      runHeadingPID();
      return;
    } else if (Robot.shuffleboard.getPracticeModeType() == PracticeModeType.TUNE_MODULE_DRIVE) {
      testDriveVel();
      return;
    } else if (Robot.shuffleboard.getPracticeModeType() == PracticeModeType.TUNE_MODULE_TURN){
      testTurnAngle();
      return;
    }

    m_swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_pigeon.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(m_swerveModuleStates, Constants.drive.kMaxSpeed);
    setModuleStates(m_swerveModuleStates);
  }


  private void runHeadingPID() {
    headingPIDOutput = rotationController.calculate(getAngleHeading(), Robot.shuffleboard.getRequestedHeading()); // should be in rad/s
    
    // headingOutput is in rad/s. Need to convert to m/s by multiplying by radius
    headingPIDOutput *= Math.sqrt(0.5) * Constants.drive.kTrackWidth;

    m_swerveModuleStates = new SwerveModuleState[] {
      new SwerveModuleState(-headingPIDOutput, new Rotation2d(Units.degreesToRadians(-45))),
      new SwerveModuleState(headingPIDOutput, new Rotation2d(Units.degreesToRadians(45))),
      new SwerveModuleState(-headingPIDOutput, new Rotation2d(Units.degreesToRadians(45))),
      new SwerveModuleState(headingPIDOutput, new Rotation2d(Units.degreesToRadians(-45)))
    };
    setModuleStates(m_swerveModuleStates);
  }

  private void testDriveVel() {
    double value = Robot.shuffleboard.getRequestedVelocity();
    m_swerveModuleStates = new SwerveModuleState[] {
      new SwerveModuleState(value, new Rotation2d(Units.degreesToRadians(135))),
      new SwerveModuleState(value, new Rotation2d(Units.degreesToRadians(45))),
      new SwerveModuleState(value, new Rotation2d(Units.degreesToRadians(225))),
      new SwerveModuleState(value, new Rotation2d(Units.degreesToRadians(315)))
    };
    setModuleStates(m_swerveModuleStates);
  }

  private void testTurnAngle() {
    double value = Robot.shuffleboard.getRequestedTurnAngle();
    m_swerveModuleStates = new SwerveModuleState[] {
      new SwerveModuleState(0, new Rotation2d(Units.degreesToRadians(value))),
      new SwerveModuleState(0, new Rotation2d(Units.degreesToRadians(value))),
      new SwerveModuleState(0, new Rotation2d(Units.degreesToRadians(value))),
      new SwerveModuleState(0, new Rotation2d(Units.degreesToRadians(value)))
    };
    setModuleStates(m_swerveModuleStates);
  }


  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(
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


  public void toggleDriveMode() {
    isSlewDrive = !isSlewDrive;
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
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
    Rotation2d angle = m_pigeon.getRotation2d();
    return Math.atan2(angle.getSin(), angle.getCos());
  }

  /**
   * Gets an array of all the swerve module positions.
   * 
   * @return an array of all swerve module positions
   */
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      positions[i] = new SwerveModulePosition();
    }
    return positions;
  }

  /**
   * Sets the desired states for all swerve modules.
   * 
   * @param swerveModuleStates an array of module states to set swerve modules to
   */
  public void setModuleStates(SwerveModuleState[] swerveModuleStates) {
    for (int i = 0; i < 4; i++) {
      m_modules[i].setDesiredState(swerveModuleStates[i]);
    }
  }

  public PIDController getXController() {
      return xController;
  }
  public PIDController getYController() {
      return yController;
  }
  public PIDController getRotationController() {
    return rotationController;
  }

}