package frc.robot.util;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.VisionConstants;

/**
 * Class to store data about scoring locations
 * Stores location of node relative to driver's perspective
*/
public class NodeLegecy {
  // Possible node types
  public enum Types {CONE, CUBE, HYBRID};
  
  // The node's alliance
  public final Alliance alliance;
  // Which row it's on (1-3)
  public final int row;
  // Which column it's in (1-9), from the driver's persective left to right
  public final int column;
  // The node's pose
  public final Pose3d pose;
  // Where the robot goes to score in the node
  public final Pose2d scorePose;
  // Which type of node it is
  public final Types type;
  
  /**
  * Creates a new Node with default values
  */
  public NodeLegecy(){
    // Uses the robot's alliance
    alliance = DriverStation.getAlliance();
    // Default values are 1st row, 1st column, pose of (0, 0, 0), score pose of (0, 0), and hybrid
    row = 1;
    column = 1;
    pose = new Pose3d();
    scorePose = new Pose2d();
    type = Types.HYBRID;
  }
  
  /**
  * Creates a new Node object
  * @param vision The vision, for april tag positions
  * @param alliance The node's color
  * @param row Which row it's in (1 = bottom, 2 = middle, 3 = top)
  * @param column Grid column from left to right (1-9)
  */
  public NodeLegecy(Vision vision, Alliance alliance, int row, int column){
    // Sets the alliance, row, and column to the parameters
    this.alliance = alliance;
    this.row = row;
    this.column = column;
    // If it's on the first row, it's a hybrid node, if it's in the middle of the grid,
    // if it's in the middle of a grid, it's a cube node,
    // and if it isn't either of those, it's a cone node
    type = row==1?Types.HYBRID : column%3==2?Types.CUBE : Types.CONE;
    
    // Which April tag this node is closest to
    // Uses integer division to return 0-2, then adds 1 for red or 6 for blue to get 1-8
    Pose3d tag = vision.getTagPose((column-1)/3+(alliance==Alliance.Red?1:6));
    // The x coordinate in meters
    double x;
    // Vertical distance in meters
    double z;

    // All numbers are from the CAD of the field
    // I'm not going to put exactly what all of them are here
    switch(row){
      case(1):
        x = tag.getX()+Units.inchesToMeters(14.25/2)*(alliance==Alliance.Red?-1:1);
        z = 0;
        break;
      case(2):
        x = tag.getX()-Units.inchesToMeters((31.625-14.25)/2)*(alliance==Alliance.Red?-1:1);
        z = Units.inchesToMeters(type==Types.CUBE?23.5:34);
        break;
      case(3):
        x = tag.getX()-Units.inchesToMeters(39.75-14.25)*(alliance==Alliance.Red?-1:1);
        z = Units.inchesToMeters(type==Types.CUBE?35.5:46);
        break;
      default:
        throw(new IllegalArgumentException("Row had to be 1, 2, or 3"));
    }
    // Y coordinate in meters
    double y = tag.getY()+(column%3==2?0:Units.inchesToMeters(22))*((column%3==0)^(alliance==Alliance.Red)?-1:1);
    // Uses the April tag's rotation and the values previously calculated
    pose = new Pose3d(x, y, z, tag.getRotation());
    // Adds the grid distance to the edge of the grid to find where the robot should be
    // The robot should be at 0 degrees (facing left) if it's blue and 180 degrees (π radians) if it's red
    scorePose = new Pose2d(tag.getX()+Units.inchesToMeters(14.25+VisionConstants.kGridDistance)*(alliance==Alliance.Red?-1:1), y, new Rotation2d(alliance==Alliance.Red?0:Math.PI));
  }
}
