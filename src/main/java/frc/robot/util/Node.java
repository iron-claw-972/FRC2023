package frc.robot.util;


import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.VisionConstants;

/**
 * Class to store data about scoring locations
 */
public class Node {

  // Possible node types
  public enum NodeType {CONE, CUBE, HYBRID};
  
  public final Alliance alliance;
  public final int row;
  public final int column;
  public final Pose2d scorePose;
  public final NodeType type;
  
  /**
   * Creates a new Node with default values
   */
  public Node()  {
    alliance = DriverStation.getAlliance();
    row = 1;
    column = 1;
    scorePose = new Pose2d();
    type = NodeType.HYBRID;
  }
  
  /**
   * Creates a new Node object
   * @param alliance
   *  the alliance color
   * @param row
   *  row it's in (1 = bottom, 2 = middle, 3 = top)
   * @param column
   *  column from field boundary to other boundary (1 to 9)
   */
  public Node(Alliance alliance, int row, int column) {

    this.alliance = alliance;
    this.row = row;
    this.column = column;

    type = (row == 1)
      ? NodeType.HYBRID
      : (column % 3 == 2)
          ? NodeType.CUBE
          : NodeType.CONE;

    // Starting locations
    double x = 0;
    if (alliance == Alliance.Blue) {
      x = VisionConstants.kAprilTags.get(5).pose.getX() + FieldConstants.kAprilTagOffset + FieldConstants.kRobotDistanceFromNodeTape;
    } else {
      x = VisionConstants.kAprilTags.get(2).pose.getX() - FieldConstants.kAprilTagOffset - FieldConstants.kRobotDistanceFromNodeTape;
    }

    double y = 0;  

    double yOffset = 0;

    // Distance from the field boundary to the boundary separating the grid and loading zone
    if (column % 3 == 0) {
      yOffset = FieldConstants.kConeNodeOffset;
    } else if (column % 3 == 1) {
      yOffset = -FieldConstants.kConeNodeOffset;
    }

    if (column <= 3) {
      y = VisionConstants.kAprilTags.get(7).pose.getY();
    } else if (column <= 6) {
      y = VisionConstants.kAprilTags.get(6).pose.getY();
    } else {
      y = VisionConstants.kAprilTags.get(5).pose.getY();
    }
    y += yOffset;

    Rotation2d rotation = new Rotation2d();
    if (alliance == Alliance.Blue) {
      rotation = new Rotation2d(Math.toRadians(180));
    }
    scorePose = new Pose2d(x, y, rotation);
  }

}