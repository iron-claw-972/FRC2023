package frc.robot.constants.swerve;

import edu.wpi.first.math.util.Units;
import frc.robot.Robot;
import frc.robot.Robot.RobotId;
import frc.robot.constants.Constants;
import frc.robot.constants.FalconConstants;

/**
 * Constants, are by default, for the competition robot. Constants get changed if the RobotId detected is not the competition robot.
 * @see RobotId
 */
public class DriveConstants {

  public static double kRobotWidthWithBumpers = Units.inchesToMeters(26 + 3.75 * 2);

  public static double kWheelRadius = Units.inchesToMeters(2);

  public static double kTrackWidth = Units.inchesToMeters(20.75);//22.75 swerve bot, 20.75 comp bot
  public static double kDriveGearRatio = 6.75;
  public static double kSteerGearRatio = 21.43;

  public static double kMaxSpeed = (FalconConstants.kMaxRpm / 60.0) / kDriveGearRatio * kWheelRadius * 2 * Math.PI;

  // Need to convert tangential velocity (the m/s of the edge of the robot) to angular velocity (the radians/s of the robot)
  // To do so, divide by the radius. The radius is the diagonal of the square chassis, diagonal = sqrt(2) * side_length.
  public static double kMaxAngularSpeed = kMaxSpeed / ((kTrackWidth/2) * Math.sqrt(2));

  // TODO: tune this better.
  public static double kMaxAngularAccel = 8 * 2 * Math.PI; // 8 rotations per second per second

  public static int kPigeon = 0;

  public static double kStartingHeadingDegrees = 0;

  public static int kDriveFrontLeft = 20;
  public static int kSteerFrontLeft = 15;
  public static int kEncoderFrontLeft = 40;
  public static double kSteerOffsetFrontLeft = 0.058291152119637;//-3.060285486280918+Math.PI;
  public static double kDriveKSFrontLeft = 0.5309532917741253;
  public static double kDriveKVFrontLeft = 2.3335800153688133;
  public static double kDrivePFrontLeft = 2*2;
  public static double kDriveIFrontLeft = 0;
  public static double kDriveDFrontLeft = 0;
  public static double kSteerKSFrontLeft = 0.7799787873000478;
  public static double kSteerKVFrontLeft = 0.37367610423475633;
  public static double kSteerPFrontLeft = 14;
  public static double kSteerIFrontLeft = 0;
  public static double kSteerDFrontLeft = 0;
  
  public static int kDriveFrontRight = 33; 
  public static int kSteerFrontRight = 30;
  public static int kEncoderFrontRight = 41; 
  public static double kSteerOffsetFrontRight = -2.994324445724487;//-3.001994334161282;
  public static double kDriveKSFrontRight = 0.705225250399035;
  public static double kDriveKVFrontRight = 2.1932870453571005;
  public static double kDrivePFrontRight = 2*2;
  public static double kDriveIFrontRight = 0;
  public static double kDriveDFrontRight = 0;
  public static double kSteerKSFrontRight = 0.7922475586517754;
  public static double kSteerKVFrontRight = 0.37485552675539596;
  public static double kSteerPFrontRight = 14;
  public static double kSteerIFrontRight = 0;
  public static double kSteerDFrontRight = 0;
  
  public static int kDriveBackLeft = 16;
  public static int kSteerBackLeft = 18;
  public static int kEncoderBackLeft = 42; 
  public static double kSteerOffsetBackLeft = -2.540267050266266;//0.650406539440155+Math.PI;
  public static double kDriveKSBackLeft = 0.486459484740409;
  public static double kDriveKVBackLeft = 2.4125110222062958;
  public static double kDrivePBackLeft = 2*2;
  public static double kDriveIBackLeft = 0;
  public static double kDriveDBackLeft = 0;
  public static double kSteerKSBackLeft = 0.7858221245795006;
  public static double kSteerKVBackLeft = 0.3836595789584776;
  public static double kSteerPBackLeft = 14;
  public static double kSteerIBackLeft = 0;
  public static double kSteerDBackLeft = 0;
  
  public static int kDriveBackRight = 32;
  public static int kSteerBackRight = 35;
  public static int kEncoderBackRight = 43; 
  public static double kSteerOffsetBackRight = 2.626169800758362;//2.771897681057453;
  public static double kDriveKSBackRight = 0.6282516293779395;
  public static double kDriveKVBackRight = 2.2812532747924257;
  public static double kDrivePBackRight = 2.2*2;
  public static double kDriveIBackRight = 0;
  public static double kDriveDBackRight = 0;
  public static double kSteerKSBackRight = 0.7914079268516959;
  public static double kSteerKVBackRight = 0.388059634963963;
  public static double kSteerPBackRight = 14;
  public static double kSteerIBackRight = 0;
  public static double kSteerDBackRight = 0;

  // heading PID
  public static double kHeadingP= 4.6;
  public static double kHeadingI= 0;
  public static double kHeadingD= 0.4;

  // CAN
  public static String kDriveMotorCAN = Constants.kCanivoreCAN;
  public static String kSteerMotorCAN = Constants.kCanivoreCAN;
  public static String kSteerEncoderCAN = Constants.kCanivoreCAN;
  public static String kPigeonCAN = Constants.kCanivoreCAN;

  /**
   * Updates the constants if the RobotId is not the competition robot.
   */
  public static void update() {
    if (Robot.kRobotId == RobotId.SwerveTest) {
  
      kTrackWidth = Units.inchesToMeters(22.75); //22.75 swerve bot, 20.75 comp bot
  
      kPigeon = 13;
  
      kDriveFrontLeft = 1;
      kSteerFrontLeft = 2;
      kEncoderFrontLeft = 3;
      kSteerOffsetFrontLeft = -1.58+Math.PI;
      kDriveKSFrontLeft = 0.61534;
      kDriveKVFrontLeft = 4.45071*2;
      kDrivePFrontLeft = 2*2;
      kDriveIFrontLeft = 0;
      kDriveDFrontLeft = 0;
      kSteerKSFrontLeft = 1;
      kSteerKVFrontLeft = 0.5;
      kSteerPFrontLeft = 12;
      kSteerIFrontLeft = 0;
      kSteerDFrontLeft = 0;
      
      kDriveFrontRight = 4; 
      kSteerFrontRight = 5;
      kEncoderFrontRight = 6; 
      kSteerOffsetFrontRight = 1.935+Math.PI;
      kDriveKSFrontRight = 0.61283;
      kDriveKVFrontRight = 4.45431*2;
      kDrivePFrontRight = 2*2;
      kDriveIFrontRight = 0;
      kDriveDFrontRight = 0;
      kSteerKSFrontRight = 1;
      kSteerKVFrontRight = 0.5;
      kSteerPFrontRight = 12;
      kSteerIFrontRight = 0;
      kSteerDFrontRight = 0;
      
      kDriveBackLeft = 7;
      kSteerBackLeft = 8;
      kEncoderBackLeft = 9; 
      kSteerOffsetBackLeft = -3.1415+Math.PI;
      kDriveKSBackLeft = 0.54150;
      kDriveKVBackLeft = 4.53909*2;
      kDrivePBackLeft = 2*2;
      kDriveIBackLeft = 0;
      kDriveDBackLeft = 0;
      kSteerKSBackLeft = 1;
      kSteerKVBackLeft = 0.5;
      kSteerPBackLeft = 12;
      kSteerIBackLeft = 0;
      kSteerDBackLeft = 0;
      
      kDriveBackRight = 10;
      kSteerBackRight = 11;
      kEncoderBackRight = 12; 
      kSteerOffsetBackRight = -0.383494421839714+Math.PI;
      kDriveKSBackRight = 0.49599;
      kDriveKVBackRight = 4.75897*2;
      kDrivePBackRight = 2.2*2;
      kDriveIBackRight = 0;
      kDriveDBackRight = 0;
      kSteerKSBackRight = 1;
      kSteerKVBackRight = 0.5;
      kSteerPBackRight = 12;
      kSteerIBackRight = 0;
      kSteerDBackRight = 0;
  
      // CAN
      kDriveMotorCAN = Constants.kRioCAN;
    }
  }

}
