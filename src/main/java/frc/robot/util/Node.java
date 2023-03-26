package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.RollerIntake;

/**
 * Class to store data about scoring locations
 * positions are stored from field border to barrier instead of left to right
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
   *  column from field boundary to loading zone (1 to 9)
   */
  public Node(RollerIntake intake, Alliance alliance, int row, int column) {

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
      x = FieldConstants.kAprilTags.get(5).pose.getX() + FieldConstants.kAprilTagOffset + VisionConstants.kGridDistanceAlignment;
    } else {
      x = FieldConstants.kAprilTags.get(2).pose.getX() - FieldConstants.kAprilTagOffset - VisionConstants.kGridDistanceAlignment;
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
      y = FieldConstants.kAprilTags.get(7).pose.getY();
    } else if (column <= 6) {
      y = FieldConstants.kAprilTags.get(6).pose.getY();
    } else {
      y = FieldConstants.kAprilTags.get(5).pose.getY();
    }
    y += yOffset;

    Rotation2d rotation = new Rotation2d();
    if (alliance == Alliance.Blue) {
      rotation = Rotation2d.fromDegrees(180);
    }
    double distanceSensorOffset = intake==null?0:(intake.getDistnace() - IntakeConstants.kCenterDist);
    y += distanceSensorOffset * (alliance==Alliance.Blue?1:-1);
    scorePose = new Pose2d(x, y, rotation);
  }
}