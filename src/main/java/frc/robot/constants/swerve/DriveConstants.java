package frc.robot.constants.swerve;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import lib.COTSFalconSwerveConstants;
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

  /* Drive Motor Characterization Values 
  * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
  public static final double kDriveKS = 0.32 / 12.0; // 0.65559
  public static final double kDriveKV = 1.51 / 12.0; // 1.93074
  public static final double kDriveKA = 0.27 / 12.0; // 0.00214

  public static double kMaxSpeed = (FalconConstants.kMaxRpm / 60.0) * kWheelRadius * 2 * Math.PI / kDriveGearRatio;

  // Need to convert tangential velocity (the m/s of the edge of the robot) to angular velocity (the radians/s of the robot)
  // To do so, divide by the radius. The radius is the diagonal of the square chassis, diagonal = sqrt(2) * side_length.
  public static double kMaxAngularSpeed = kMaxSpeed / ((kTrackWidth/2) * Math.sqrt(2));

  // TODO: tune this better.
  public static double kMaxAngularAccel = 8 * 2 * Math.PI; // 8 rotations per second per second

  public static int kPigeon = 0;

  public static double kStartingHeadingDegrees = 180;

  public static final SwerveDriveKinematics kKinematics = new SwerveDriveKinematics(
    new Translation2d(DriveConstants.kTrackWidth / 2, DriveConstants.kTrackWidth / 2),
    new Translation2d(DriveConstants.kTrackWidth / 2, -DriveConstants.kTrackWidth / 2),
    new Translation2d(-DriveConstants.kTrackWidth / 2, DriveConstants.kTrackWidth / 2),
    new Translation2d(-DriveConstants.kTrackWidth / 2, -DriveConstants.kTrackWidth / 2)
  );

  public static int kDriveFrontLeft = 20;
  public static int kSteerFrontLeft = 15;
  public static int kEncoderFrontLeft = 40;
  public static double kSteerOffsetFrontLeft = 0.058291152119637;//-3.060285486280918+Math.PI;
  public static double kDriveKSFrontLeft = 0.5309532917741253;
  public static double kDriveKVFrontLeft = 2.3335800153688133;
  public static double kDrivePFrontLeft = 5.5;
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
  public static double kDrivePFrontRight = 5.5;
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
  public static double kDrivePBackLeft = 5.5;
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
  public static double kDrivePBackRight = 5.5;
  public static double kDriveIBackRight = 0;
  public static double kDriveDBackRight = 0;
  public static double kSteerKSBackRight = 0.7914079268516959;
  public static double kSteerKVBackRight = 0.388059634963963;
  public static double kSteerPBackRight = 14;
  public static double kSteerIBackRight = 0;
  public static double kSteerDBackRight = 0;

  // heading PID
  public static double kHeadingP= 1;//4
  public static double kHeadingD= 0;

  //balance PID
  public static double kBalanceP = 0.05;
  public static double kBalanceI = 0;
  public static double kBalanceD = 0;
  // max output in m/s
  public static final double kBalanceMaxOutput = 0.5;

  //balance timer
  // when the balance command starts, it will run the pid normally for
  // kBalanceNoStopPeriod seconds. Then, every kBalanceStopInterval seconds,
  // it will stop the motors for kBalanceStopDuration seconds to help give
  // the charge station some time to balance.
  // "A non-linear system requires non-linear control" - jerry
  // "If it's stupid but it works..."
  public static double kBalanceNoStopPeriod = 2;
  public static double kBalanceStopInterval = 0.5;
  public static double kBalanceStopDuration = 0.2;

  //translational PID
  public static double kTranslationalP = 0.25;
  public static double kTranslationalD = 0;//0.001

  //The PIDs for PathPlanner Command
  public static double kPathplannerHeadingP = 0.02;
  public static double kPathplannerHeadingD = 0;
  
  public static double kPathplannerTranslationalP = 0;
  public static double kPathplannerTranslationalD = 0;

  // CAN
  public static String kDriveMotorCAN = Constants.kCanivoreCAN;
  public static String kSteerMotorCAN = Constants.kCanivoreCAN;
  public static String kSteerEncoderCAN = Constants.kCanivoreCAN;
  public static String kPigeonCAN = Constants.kCanivoreCAN;
  

  public static final COTSFalconSwerveConstants kModuleConstants = COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L2);

  /* Swerve Current Limiting */
  public static final int kAngleContinuousCurrentLimit = 25;
  public static final int kAnglePeakCurrentLimit = 40;
  public static final double kAnglePeakCurrentDuration = 0.1;
  public static final boolean kAngleEnableCurrentLimit = true;

  public static final int kDriveContinuousCurrentLimit = 35;
  public static final int kDrivePeakCurrentLimit = 60;
  public static final double kDrivePeakCurrentDuration = 0.1;
  public static final boolean kDriveEnableCurrentLimit = true;

  /* Motor inversions */
  public static final boolean kDriveMotorInvert = true;//kModuleConstants.driveMotorInvert;
  public static final boolean kAngleMotorInvert = kModuleConstants.angleMotorInvert;

  /* Neutral Modes */
  public static final NeutralMode kDriveNeutralMode = NeutralMode.Brake;
  public static final NeutralMode kAngleNeutralMode = NeutralMode.Coast;

  /* Drive Motor PID Values */
  public static final double kDriveP = 0.05;
  public static final double kDriveI = 0.0;
  public static final double kDriveD = 0.0;
  public static final double kDriveF = 0.0;

  /* Ramp values for drive motors in open and closed loop driving. */
  // Open loop prevents throttle from changing too quickly. 
  // It will limit it to time given (in seconds) to go from zero to full throttle.
  // A small open loop ramp (0.25) helps with tread wear, tipping, etc
  public static final double kOpenLoopRamp = 0.1;
  public static final double kClosedLoopRamp = 0.0;

  public static final double kWheelCircumference = kModuleConstants.wheelCircumference;
  
  /* Motor gear ratios */
  public static final double kAngleGearRatio = kModuleConstants.angleGearRatio;

  public static final boolean kInvertGyro = false; // Make sure gyro is CCW+ CW- // FIXME: Swerve
  
  public static final double kSlowDriveFactor = 0.3;
  public static final double kSlowRotFactor = 0.1;

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
      kSteerKSFrontLeft = 0.7380365018530551;
      kSteerKVFrontLeft = 0.3777356557786064;
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
      kSteerKSFrontRight = 0.6206519356062554;
      kSteerKVFrontRight = 0.36589750785689507;
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
      kSteerKSBackRight = 0.7506108061783936;
      kSteerKVBackRight = 0.37750397997126794;
      kSteerPBackRight = 12;
      kSteerIBackRight = 0;
      kSteerDBackRight = 0;
  
      // CAN
      kDriveMotorCAN = Constants.kRioCAN;
    }
  }

}
