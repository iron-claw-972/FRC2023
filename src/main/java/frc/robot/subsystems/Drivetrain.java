/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.Optional;

// import org.apache.commons.lang3.tuple.Pair;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.DriveConstants;
import frc.robot.util.MotorFactory;
import frc.robot.util.Vision;
import lib.ctre_shims.TalonEncoder;

public class Drivetrain extends SubsystemBase {

  private final WPI_TalonFX m_leftMotor1;
  private final WPI_TalonFX m_leftMotor2;
  private final WPI_TalonFX m_rightMotor1;
  private final WPI_TalonFX m_rightMotor2;

  private final TalonEncoder m_leftEncoder;
  private final TalonEncoder m_rightEncoder;

  private final DifferentialDrivePoseEstimator m_poseEstimator;

  private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(DriveConstants.kTrackWidth);
  private final AHRS m_gyro;


  public Drivetrain(){
    this(new AHRS(SPI.Port.kMXP));
  }

  public Drivetrain(AHRS gyro) {

    m_leftMotor1 = MotorFactory.createTalonFX(DriveConstants.kLeftMotor1, Constants.kRioCAN);
    m_leftMotor2 = MotorFactory.createTalonFX(DriveConstants.kLeftMotor2, Constants.kRioCAN);
    m_rightMotor1 = MotorFactory.createTalonFX(DriveConstants.kRightMotor1, Constants.kRioCAN);
    m_rightMotor2 = MotorFactory.createTalonFX(DriveConstants.kRightMotor2, Constants.kRioCAN);

    SupplyCurrentLimitConfiguration supplyCurrentLimit = new SupplyCurrentLimitConfiguration(true, 40, 45, 1);

    m_leftMotor1.configSupplyCurrentLimit(supplyCurrentLimit);
    m_leftMotor2.configSupplyCurrentLimit(supplyCurrentLimit);
    m_rightMotor1.configSupplyCurrentLimit(supplyCurrentLimit);
    m_rightMotor2.configSupplyCurrentLimit(supplyCurrentLimit);

    m_leftMotor2.follow(m_leftMotor1);
    m_rightMotor2.follow(m_rightMotor1);

    // Encoder setup
    m_leftEncoder = new TalonEncoder(m_leftMotor1);
    m_rightEncoder = new TalonEncoder(m_rightMotor1);

    // Gyro setup
    m_gyro = gyro;
    resetGyro();

    m_poseEstimator = new DifferentialDrivePoseEstimator(m_kinematics, m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance(), new Pose2d(new Translation2d(15.05, 2.79), new Rotation2d(0)), new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, 0.01), new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.1, 0.1, 0.01));
  }

  /**
   * Drives the robot using tank drive controls Tank drive is slightly easier to code but less
   * intuitive to control, so this is here as an example for now
   *
   * @param leftPower the commanded power to the left motors
   * @param rightPower the commanded power to the right motors
   */
  public void tankDrive(double leftPower, double rightPower) {
    m_leftMotor1.set(ControlMode.PercentOutput, leftPower);
    m_rightMotor1.set(ControlMode.PercentOutput, rightPower);
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param forward the commanded forward movement
   * @param turn the commanded turn rotation
   */
  public void arcadeDrive(double throttle, double turn) {
    m_leftMotor1.set(ControlMode.PercentOutput, throttle + turn);
    m_rightMotor1.set(ControlMode.PercentOutput, throttle - turn);
  }

  /**
   * Updates the robot's pose using encoders, gyro, and vision
   */
  public void updateOdometry() {
    // Upate robot pose (x, y, theta)
    m_poseEstimator.update(
      m_gyro.getRotation2d(),
      m_leftEncoder.getDistance(),
      m_rightEncoder.getDistance()
    );
    Optional<Pair<Pose3d, Double>> result = Vision.getEstimatedGlobalPose(m_poseEstimator.getEstimatedPosition());

    if (result.isPresent() && result.get().getFirst() != null && result.get().getSecond() != null && result.get().getFirst().getX() > -1000 && result.get().getSecond() >= 0) {
      Pair<Pose3d,Double> camPose = result.get();
      m_poseEstimator.addVisionMeasurement(camPose.getFirst().toPose2d(), Timer.getFPGATimestamp() - Units.millisecondsToSeconds(camPose.getSecond()));
      // m_poseEstimator.addVisionMeasurement(new Pose2d(), 0.02);
      // System.out.println(camPose.getFirst().toPose2d().toString());
    }
  }

  public double getLeftDistance(){
    return m_leftEncoder.getDistance();
  }
  public double getRightDistance(){
    return m_rightEncoder.getDistance();
  }

  public Pose2d getPose(){
    return m_poseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the gyro
   */
  public void resetGyro() {
    m_gyro.reset();
  }

  /**
   * Sets the robot's position
   * @param x The x coordinate
   * @param y The y coordinate
   * @param rotation The rotation in radians
   */
  public void resetPose(double x, double y, double rotation){
    m_poseEstimator.resetPosition(m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance(), new Pose2d(new Translation2d(x, y), new Rotation2d(rotation)));
  }

  /**
   * Prints out the robot's current pose
   */
  public void printPose(){
    Pose2d p = m_poseEstimator.getEstimatedPosition();
    System.out.println(Vision.getTagFieldLayout().getTagPose(2).toString());
    System.out.printf("ROBOT POSE:\ntoString(): %s\nRotation: %.2f degrees\nPosition: (%.2f, %.2f)\n", p.toString(), p.getRotation().getDegrees(), p.getX(), p.getY());
  }
}
