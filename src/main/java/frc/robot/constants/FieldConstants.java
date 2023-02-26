package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public class FieldConstants {
  public static final double kFieldLength = Units.inchesToMeters(54*12 + 3.25); // meters
  public static final double kFieldWidth = Units.inchesToMeters(26*12 + 3.5); // meters

  // The distance from the node tape to the edge of the robot's bumpers
  // TODO: Change this!
  public static final double kDistanceFromNodeTape = 1.0; // meters

  public static final double kBlueAllianceNodeStartX = kDistanceFromNodeTape + 4.379; // meters
  public static final double kRedAllianceNodeStartX = kBlueAllianceNodeStartX + 13.804 - kDistanceFromNodeTape; // meters

  public static final double kNodeStartY = 0.569; // meters

}
