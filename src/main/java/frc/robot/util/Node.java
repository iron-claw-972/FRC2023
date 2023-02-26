package frc.robot.util;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.Constants;
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
  public Node(Vision vision, Alliance alliance, int row, int column) {

    this.alliance = alliance;
    this.row = row;
    this.column = column;

    type = (row == 1)
      ? NodeType.HYBRID
      : (column % 3 == 2)
          ? NodeType.CUBE
          : NodeType.CONE;

    // The 
    double x = (alliance == Alliance.Blue ? FieldConstants.kBlueAllianceNodeStartX : FieldConstants.kRedAllianceNodeStartX);
    double y = FieldConstants.kNodeStartY;

    // Distance from the field boundary to the boundary separating the grid and loading zone
    switch (column) {
      case 1:
        y += 0.508;  
        break;
      case 2:
        y += 1.056;;  
        break;
      case 3:
        y += 1.613;  
        break;
      case 4:
        y += 2.182;
        break;
      case 5:
        y += 2.732;  
        break;
      case 6:
        y += 3.302;  
        break;
      case 7:
        y += 3.861;  
        break;
      case 8:
        y += 4.409;  
        break;
      case 9:
        y += 4.978;  
        break;
    }
    scorePose = new Pose2d(x, y, new Rotation2d());
  }

}