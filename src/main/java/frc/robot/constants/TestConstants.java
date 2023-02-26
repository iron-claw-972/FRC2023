package frc.robot.constants;

import edu.wpi.first.math.util.Units;

/**
 * Container class for test constants.
 */
public class TestConstants {
  
  public static final double kTranslationError = 0;
  //time error for odometry is dependent on test
  public static final double kHeadingError = Units.degreesToRadians(1);
  public static final double kHeadingTimeError = 0.1;
  
  public static final double kSteerAngleError = Units.degreesToRadians(1);
  public static final double kSteerAngleTimeError = 0.1; 
  public static final double kDriveVelocityError = 0.1;
  public static final double kDriveVelocityTimeError = 0.1;

  public static final double kDriveFeedForwardVoltageStep = 0.2;
  public static final double kDriveFeedForwardMaxVoltage = 11;
  public static final double kDriveFeedForwardAccelerationTimeBuffer = 0.5;
  public static final double kDriveFeedForwardRecordingTime = 1.5;
  
  public static final double kSteerFeedForwardVoltageStep = 0.2;
  public static final double kSteerFeedForwardMaxVoltage = 6;
  public static final double kSteerFeedForwardAccelerationTimeBuffer = 0.5;
  public static final double kSteerFeedForwardRecordingTime = 2;
  
}