package frc.robot.constants;

import edu.wpi.first.math.util.Units;
import frc.robot.constants.swerve.DriveConstants;

public class FieldConstants {
  public static final double kFieldLength = Units.inchesToMeters(54*12 + 3.25); // meters
  public static final double kFieldWidth = Units.inchesToMeters(26*12 + 3.5); // meters

  // The distance from the node tape to the edge of the robot's bumpers, for node scoring
  // TODO: 1.0 is just a dummy value at the moment, so this value needs to be changed to a proper value
  public static final double kDistanceFromNodeTape = 1.0 + DriveConstants.kRobotWidthWithBumpers / 2; // meters

  public static final double kBlueAllianceNodeStartX = kDistanceFromNodeTape + 4.379; // meters
  public static final double kRedAllianceNodeStartX = kBlueAllianceNodeStartX + 13.804 - kDistanceFromNodeTape; // meters

  public static final double kNodeStartY = 0.569; // meters

  // TODO: This value needs to be tested to ensure functionality, for node scoring
  public static final double kRobotOffsetY = DriveConstants.kRobotWidthWithBumpers / 2; // meters

}
