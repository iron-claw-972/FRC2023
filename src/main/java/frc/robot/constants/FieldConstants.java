package frc.robot.constants;

import edu.wpi.first.math.util.Units;
import frc.robot.constants.swerve.DriveConstants;

public class FieldConstants {
  public static final double kFieldLength = Units.inchesToMeters(54*12 + 3.25); // meters
  public static final double kFieldWidth = Units.inchesToMeters(26*12 + 3.5); // meters

  // The distance from the node tape to the edge of the robot's bumpers, for node scoring
  // TODO: After testing, this value might need to be tuned. Change the 0+ to however much the robot needs to be away from the node tape
  public static final double kRobotDistanceFromNodeTape = 0 + DriveConstants.kRobotWidthWithBumpers / 2; // meters

  // The locations of the node tape in the x axis, relative to the very bottom left corner of the entire field
  public static final double kBlueNodeTapePosX = 4.379; // meters
  public static final double kRedNodeTapePosX = 18.183; // meters

  // The location that the robot should be at in the X axis
  public static final double kBlueAllianceNodeStartX = kBlueNodeTapePosX + kRobotDistanceFromNodeTape; // meters
  // Subtract because the red alliance is on the opposite side of the field
  public static final double kRedAllianceNodeStartX = kRedNodeTapePosX - kRobotDistanceFromNodeTape; // meters

  // The location of the field boundary in the y axis, relative to the very bottom left corner of the entire field
  public static final double kNodeStartY = 0.569; // meters

  // TODO: This value needs to be tested to ensure functionality, for node scoring
  public static final double kRobotOffsetX = DriveConstants.kRobotWidthWithBumpers / 2; // meters

  // Where the robot needs to go to intake from the shelf
  public static double kBlueShelfX = VisionConstants.kAprilTags.get(3).pose.getX()-Units.inchesToMeters(VisionConstants.kShelfDistance);
  public static double kRedShelfX = VisionConstants.kAprilTags.get(4).pose.getX()+Units.inchesToMeters(VisionConstants.kShelfDistance);
  public static double kShelfY = VisionConstants.kAprilTags.get(3).pose.getY();
}
