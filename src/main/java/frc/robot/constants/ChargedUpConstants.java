package frc.robot.constants;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

/**
 * Constants related to the ChargedUp game.
 * <p>
 * For example, the dimensions of the field and the location of field elements.
 */
public class ChargedUpConstants {
    // The field dimensions
    /** FRC Field Length (in meters) */
    public static final double fieldLength = Units.feetToMeters(54);
    /** FRC Field Width (in meters) */
    public static final double fieldWidth = Units.feetToMeters(27);

    // The location of the April Tags.
    // see https://first.wpi.edu/wpilib/allwpilib/docs/development/cpp/classfrc_1_1_april_tag_field_layout.html
    //
    // build the April Tags from FRC documentation
    // 
    private static final Rotation3d rot000 = new Rotation3d(0.0, 0.0, 0.0);
    private static final Rotation3d rot180 = new Rotation3d(0.0, 0.0, Math.PI);
    /**
     * April Tag locations from FRC document<p>
     * See https://firstfrc.blob.core.windows.net/frc2023/Manual/TeamUpdates/TeamUpdate01.pdf <p>
     * <table>
     * <tr><th>ID</th><th>X</th><th>Y</th><th>Z</th><th>Z-rotation</th></tr>
     * <tr><td>1</td><td> 610.77</td><td>  42.19</td><td>  18.22</td><td>  180</td></tr>
     * <tr><td>2</td><td> 610.77</td><td> 108.19</td><td>  18.22</td><td>  180</td></tr>
     * <tr><td>3</td><td> 610.77</td><td> 174.19</td><td>  18.22</td><td>  180</td></tr>
     * <tr><td>4</td><td> 636.96</td><td> 265.74</td><td>  27.38</td><td>  180</td></tr>
     * <tr><td>5</td><td>  14.25</td><td> 265.74</td><td>  27.38</td><td>    0</td></tr>
     * <tr><td>6</td><td>  40.45</td><td> 174.19</td><td>  18.22</td><td>    0</td></tr>
     * <tr><td>7</td><td>  40.45</td><td> 108.19</td><td>  18.22</td><td>    0</td></tr>
     * <tr><td>8</td><td>  40.45</td><td>  42.19</td><td>  18.22</td><td>    0</td></tr>
     * </table>
     */
    private static final ArrayList<AprilTag> aprilTags = new ArrayList<AprilTag>(List.of(
        new AprilTag(1, new Pose3d(Units.inchesToMeters(610.77), Units.inchesToMeters( 42.19), Units.inchesToMeters(18.22), rot180)),
        new AprilTag(2, new Pose3d(Units.inchesToMeters(610.77), Units.inchesToMeters(108.19), Units.inchesToMeters(18.22), rot180)),
        new AprilTag(3, new Pose3d(Units.inchesToMeters(610.77), Units.inchesToMeters(174.19), Units.inchesToMeters(18.22), rot180)),
        new AprilTag(4, new Pose3d(Units.inchesToMeters(636.96), Units.inchesToMeters(265.74), Units.inchesToMeters(27.38), rot180)),

        new AprilTag(5, new Pose3d(Units.inchesToMeters( 14.25), Units.inchesToMeters(265.74), Units.inchesToMeters(27.38), rot000)),
        new AprilTag(6, new Pose3d(Units.inchesToMeters( 40.45), Units.inchesToMeters(174.19), Units.inchesToMeters(18.22), rot000)),
        new AprilTag(7, new Pose3d(Units.inchesToMeters( 40.45), Units.inchesToMeters(108.19), Units.inchesToMeters(18.22), rot000)),
        new AprilTag(8, new Pose3d(Units.inchesToMeters( 40.45), Units.inchesToMeters( 42.19), Units.inchesToMeters(18.22), rot000))  ));

    // The field dimensions will come into play if the Alliance is switched?
    // i.e. aprilTagFieldLayoutX.setOrigin(OriginPosition.) kBlueAllianceWallRightSide kRedAllianceWallRightSide

    /**
     * Location of the April Tags for the FRC ChargedUp game.
     * <p>
     * The field dimensions come into play when the Alliance origin is shifted?
     * i.e. aprilTagFieldLayout.setOrigin(OriginPosition.) kBlueAllianceWallRightSide kRedAllianceWallRightSide
     */
    public static final AprilTagFieldLayout aprilTagFieldLayout = new AprilTagFieldLayout(aprilTags, fieldLength, fieldWidth);

    // other methods of getting the April Tag locations (execute in environment that can handle IOException)
    //   AprilTagFieldLayout aprilTagFieldLayoutRapidReact = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2022RapidReact.m_resourceFile);
    //   AprilTagFieldLayout aprilTagFieldLayoutDefault = AprilTagFieldLayout.loadFromResource(AprilTagFields.kDefaultField.m_resourceFile);

}
