package frc.robot.constants;

import edu.wpi.first.math.util.Units;
import frc.robot.constants.swerve.DriveConstants;

public class FieldConstants {
  public static final double kFieldLength = Units.inchesToMeters(54*12 + 3.25); // meters
  public static final double kFieldWidth = Units.inchesToMeters(26*12 + 3.5); // meters

  // The distance from the center of the field to any one of the staged game pieces in the x direction.
  // This value was obtained using the CAD of the field.
  public static final double kCenterToStagedPieceX = Units.inchesToMeters(47.361); // meters

  // The distance from the node tape to the edge of the robot's bumpers, for node scoring
  // TODO: After testing, this value might need to be tuned. Change the 0+ to however much the robot needs to be away from the node tape
  public static final double kRobotDistanceFromNodeTape = 0 + DriveConstants.kRobotWidthWithBumpers / 2; // meters

  // The distance from the apriltag to the edge of the tape in front of the grids
  public static final double kAprilTagOffset = Units.inchesToMeters(14.06 - 0.26); // meters

  // The distance from center of apriltag to the cone node
  public static final double kConeNodeOffset = Units.inchesToMeters(22); // meters

  // Where the robot needs to go to intake from the shelf
  public static double kBlueShelfX = VisionConstants.kAprilTags.get(3).pose.getX()-Units.inchesToMeters(VisionConstants.kShelfDistance);
  public static double kRedShelfX = VisionConstants.kAprilTags.get(4).pose.getX()+Units.inchesToMeters(VisionConstants.kShelfDistance);
  public static double kShelfY = VisionConstants.kAprilTags.get(3).pose.getY()-1.25;
}
