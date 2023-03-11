package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.util.Node.NodeType;

/**
 * Make sure that various field dimensions make sense.
 * 
 * <p>This test should run everytime someone builds the robot code.
 * See https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/unit-testing.html
 * 
 * <p>To disable a test, annotate with Disabled
 */

public class FieldElementTest {
    /** Field height is 26 feet 3.5 inches */
    double heightField = Units.inchesToMeters(26 * 12 + 3.5);
    /** Field width is 54 feet 3.25 inches */
    double widthField = Units.inchesToMeters(54 * 12 + 3.25);

    // all three grid units are 18 feet 0.5 inches (Game Manual)
    double wholeGrid = 18.0 * 12 + 0.5;

    // The outer Grid is 6 feet 3 inches (Game Manual)
    double outerGrid = 6.0 * 12 + 3;
    // The inner Grid is 5 feet 6 inches (Game Manual)
    double innerGrid = 5.0 * 12 + 6;

    // Outermost hybrid nodes are 2 feet 1.75 inches wide (Game Manual)
    double outerHybrid = 2.0 * 12 + 1.75;
    // other hybrid nodes are 1 feet 6.5 inches wide (Game Manual)
    double innerHybrid = 1.0 * 12 + 6.5;
    // width of the bar at the foot of grid (consistent value chosen from Game Manual dimensions)
    // drawings call them Dividers
    double widthBar = 3.5;

    @BeforeEach
    public void prepare() {
    }
  
    @AfterEach
    public void cleanup() {}

    /**
     * Check field dimensions
     */
    @Test
    public void testFieldDimensions() {
        // TODO: terminology: kFieldWidth is confusing because field is drawn landscape, so width != length
        assertEquals(heightField, FieldConstants.kFieldWidth, 0.001);
        assertEquals(widthField, FieldConstants.kFieldLength, 0.001);
    }

    /**
     * Check dimensions of the Grid
     */
    @Test
    public void testGridWidth() {
        // Test the width of 3 Grids (2 outer and 1 inner)
        // TODO: width of 3 Gris is 0.5 inches off
        assertEquals(wholeGrid, 2 * outerGrid + innerGrid, 0.501);

        // Test the width of an outer grid
        // bar + outer + bar + inner + bar + inner + 0.5 bar = 6 feet 3 inches == outerGrid
        // looking for 6 feet 3 inches = 75 inches
        // outerHybrid = 25.75
        // 2 * innerHyrid = 37.0
        // total hybrid is 62.75 leaves diff of 12.25
        assertEquals(outerGrid, 3.5 * widthBar + outerHybrid + 2.0 * innerHybrid, 0.001);

        // Test the width of an inner grid
        // 0.5 bar + inner + bar + inner + bar + inner + 0.5 bar = 5 feet 6 inches == innerGrid
        // looking for 5 feet 6 inches = 66 inches
        // 3 innerHybrid = 55.5 inches leaves diff of 10.5
        assertEquals(innerGrid, 3.0 * widthBar + 3.0 * innerHybrid, 0.001);
    }

    /**
     * The y-coordinates of the cube node should match the AprilTag y-coordinates
     */
    @Test
    public void cubeNodeTest() {
        // the AprilTag y values are
        AprilTag atag1 = VisionConstants.kAprilTags.get(0);
        AprilTag atag2 = VisionConstants.kAprilTags.get(1);
        AprilTag atag3 = VisionConstants.kAprilTags.get(2);

        double y1 = Units.metersToInches(atag1.pose.getY());
        double y2 = Units.metersToInches(atag2.pose.getY());
        double y3 = Units.metersToInches(atag3.pose.getY());

        // sanity check
        assertEquals(42.19, y1, 0.001);
        assertEquals(108.19, y2, 0.001);
        assertEquals(174.19, y3, 0.001);

        // delta distance is 5 feet 6 inches = 66 inches (size of inner Grid)
        // other AprilTags should be that distance away from the center tag
        assertEquals(y2-66, y1, 0.001);
        assertEquals(y2+66, y3, 0.001);

        // so now just calculate where the middle of the cube node would be
        // outerGrid is 75
        // 0.5 innerGrid is 33
        // that gives 108.0. So the 0.19 is from what? Rail overhang?
        assertEquals(y2, outerGrid + 0.5 * innerGrid + 0.19, 0.001);

        // The cone node y-coordinates will be slightly offset from the cube values
        double delta = innerHybrid + widthBar;

        // tolerance -- 1 mm is close enough
        double eps = 0.001;

        // for each column (0 to 8)
        for (int i = 0; i < 9; i++) {
            double y = 0.0;

            // set y to the AprilTag y depending upon which of 3 grids
            switch (i / 3) {
                case 0: y = y1; break;
                case 1: y = y2; break;
                case 2: y = y3; break;
            }

            // possible subtract or add delta if not the center of the grid
            switch (i % 3) {
                case 0: y -= delta; break;
                case 1: break;
                case 2: y += delta; break;
            }

            // check each row
            // TODO: verify the x value of 71 inches for Blue starting pose.
            double x = 71.0;
            for (int row = 1; row <= 3; row++) {
                // Calculate the node type
                // row 1 is HYBRID, row 2 and 3 may be CUBE or CONE
                NodeType nt = (row == 1) ? NodeType.HYBRID : (((i%3) == 1) ? NodeType.CUBE : NodeType.CONE);

                // get the Blue node
                Node nodeBlue = new Node(Alliance.Blue, row, i+1);
                // check its y value
                assertEquals(Units.inchesToMeters(y), nodeBlue.scorePose.getY(), eps);
                assertEquals(nt, nodeBlue.type);
                // check its x value
                // System.out.println(Units.metersToInches(nodeBlue.scorePose.getX()));
                assertEquals(Units.inchesToMeters(x), nodeBlue.scorePose.getX(), eps);
                
                // get the Red node
                Node nodeRed = new Node(Alliance.Red, row, i+1);
                // check its y value
                assertEquals(Units.inchesToMeters(y), nodeRed.scorePose.getY(), eps);
                assertEquals(nt, nodeRed.type);
                // check its x value
                // This value is off by 0.03 inches -- about 1 mm.
                // System.out.println(Units.metersToInches(FieldConstants.kFieldLength - nodeRed.scorePose.getX()));
                assertEquals(FieldConstants.kFieldLength - Units.inchesToMeters(x), nodeRed.scorePose.getX(), eps);
            }
        }
    }

    @Test
    public void testBlueNodeRotation() {
        // The scorePose should have the correct rotation

        Node n = new Node( Alliance.Blue, 2, 5);
        double ang = n.scorePose.getRotation().getRadians() - Math.PI;
        ang = Math.IEEEremainder(ang, 2 * Math.PI);
        assertEquals(0.0, ang, 0.000001);
    }
      
    @Test
    public void testRedNodeRotation() {
        // The scorePose should have the correct rotation

        Node n = new Node(Alliance.Red, 2, 5);
        double ang = n.scorePose.getRotation().getRadians();
        ang = Math.IEEEremainder(ang, 2 * Math.PI);
        assertEquals(0.0, ang, 0.000001);
    }

    /**
     * Transform a Blue Alliance pose to a Red Alliance pose.
     * 
     * <p> This should be a library method.
     * 
     * <p> Translate and negate x-coordinate.
     * <p> Copy y-coordinate.
     * <p> mirror rotation.
     * 
     * <p> The Pose2d object does not have a mirror operation.
     * It is just a 2-d position and a rotation.
     * @param pose the Blue Alliance pose
     * @return the Red Alliance pose
     */
    public Pose2d mirrorPoseAboutFieldCenter(Pose2d pose) {
        // mirror the x-coordinate about the center of the field
        double x = widthField - pose.getX();
        // the y-coordinate stays the same
        double y = pose.getY();
        // The angle from positive x-axis is changed to be minus from negative x-axis
        double ang = pose.getRotation().getRadians();
        Rotation2d r = new Rotation2d(Math.PI - ang);

        return new Pose2d(x, y, r);
    }

    /**
     * Possibly mirror a Blue Alliance pose
     * @param pose
     * @param alliance
     * @return
     */
    public Pose2d mirrorPoseAlliance(Pose2d poseBlue, Alliance alliance) {
        switch (alliance) {
            default:
            case Blue: return poseBlue;
            case Red: return mirrorPoseAboutFieldCenter(poseBlue);
        }
    }

    /**
     * Test poses for numerical equivalence.
     * @param p1 first pose
     * @param p2 second pose
     * @param eps tolerance for floating point comparison
     * @returns true if the poses are close enough
     */
    public boolean testEquals(Pose2d p1, Pose2d p2, double eps) {
        return Math.abs(p1.getX() - p2.getX()) < eps &&
            Math.abs(p1.getY() - p2.getY()) < eps &&
            Math.abs(Math.IEEEremainder(p1.getRotation().getRadians() - p2.getRotation().getRadians(), 2 * Math.PI)) < eps;
    }

    /**
     * Basic tests of field mirroring.
     */
    @Test
    public void testMirror() {
        // make a test pose
        Pose2d pose = new Pose2d(2.1, 3.6, new Rotation2d(Math.PI/4));
        // mirror that pose
        Pose2d poseCheck = mirrorPoseAboutFieldCenter(pose);
        // calculate the expected pose
        Pose2d poseExpect = new Pose2d(widthField - 2.1, 3.6, new Rotation2d(0.75 * Math.PI));
 
        // broken out tests
        assertEquals(poseExpect.getX(), poseCheck.getX(), 0.001);
        assertEquals(poseExpect.getY(), poseCheck.getY(), 0.001);
        assertEquals(poseExpect.getRotation().getRadians(), poseCheck.getRotation().getRadians(), 0.001);

        // integrated test
        assertTrue(testEquals(poseCheck, poseExpect, 0.001));
    }

    /**
     * Use AprilTags to check field mirroring.
     */
    @Test
    public void testFieldFlipping() {
        // The AprilTags should mirror, so use them for testing
        // Game Manual figure 5-31
        // Blue Red
        // 8 - 1
        // 7 - 2
        // 6 - 3
        // 5 - 4

        for (int i = 1; i <= 8; i++) {
            int j = 9 - i;

            // get tag i
            Pose2d poseBlue = VisionConstants.kAprilTags.get(i-1).pose.toPose2d();
            // should mirror to tag j
            Pose2d poseRed = VisionConstants.kAprilTags.get(j-1).pose.toPose2d();

            // System.out.printf("Test %d %d\n", i, j);
            // System.out.printf("   %8f %8f %8f\n", poseBlue.getX(), poseBlue.getY(), poseBlue.getRotation().getDegrees());
            // System.out.printf("   %8f %8f %8f\n", widthField - poseRed.getX(), poseRed.getY(), poseRed.getRotation().getDegrees());

            // AprilTags 4 and 5 are off by 1.2 mm
            assertTrue(testEquals(poseRed, mirrorPoseAboutFieldCenter(poseBlue), 0.002));

            // test the Alliance mirroring
            assertTrue(testEquals(poseBlue, mirrorPoseAlliance(poseBlue, Alliance.Blue), 0.002));
            assertTrue(testEquals(poseRed, mirrorPoseAlliance(poseBlue, Alliance.Red), 0.002));
        }
    }

    /**
     * Test the AprilTag z-coordinate values.
     */
    @Test
    public void testAprilTagZ() {
        // The AprilTag is 10.5 inches by 10.5 inches.
        double heightAprilTag = 10.5;

        // AprilTag z-coordinates

        // Grid AprilTags
        //   File says 18.22 inches above the carpet.
        //
        //   Field Drawings say 12.91 + 0.5 * 10.50 = 18.16 which is not equal to 18.22 but close enough
        assertEquals(18.22, 12.91 + 0.5 * heightAprilTag, 0.07);

        // TODO: Need to check field element mounting of April Tag. Should be 14.25 inches to screw position rather than bottom.
        //   Game Manual says 14.25 + 0.5 * 10.50 = 19.50
        //     Game Manual shows screw position rather than bottom of tag?
        //     AndyMark shows the large hole is centered at 0.63 from bottom; slot center ends 0.75 inches higher
        //       so 19.50 - 1.38 = 18.12
        assertEquals(18.22, 14.25 + 0.5 * heightAprilTag - 0.63 - 0.75, 0.11);


        // Substation AprilTags
        //   File says 27.38 inches above the carpet.
        //
        //   Game Manual says 1 foot 11 3/8 = 23.375 from carpet to bottom of tag
        //     thus 23.375 + 5.25 = 28.625
        //     Game Manual is showing center of the mounting screw?
        //     AndyMark shows the large hole is 0.63; the slot center ends 0.75 inches higher
        //       so subtract 1.38 from 28.625 to get 27.245
        assertEquals(27.38, 23.375 + 0.5 * heightAprilTag - 0.63 - 0.75, 0.15);
        
        // Check the simple values
        assertEquals(Units.inchesToMeters(18.22), VisionConstants.kAprilTags.get(0).pose.getZ(), 0.001);
        assertEquals(Units.inchesToMeters(18.22), VisionConstants.kAprilTags.get(1).pose.getZ(), 0.001);
        assertEquals(Units.inchesToMeters(18.22), VisionConstants.kAprilTags.get(2).pose.getZ(), 0.001);
        assertEquals(Units.inchesToMeters(27.38), VisionConstants.kAprilTags.get(3).pose.getZ(), 0.001);
        assertEquals(Units.inchesToMeters(27.38), VisionConstants.kAprilTags.get(4).pose.getZ(), 0.001);
        assertEquals(Units.inchesToMeters(18.22), VisionConstants.kAprilTags.get(5).pose.getZ(), 0.001);
        assertEquals(Units.inchesToMeters(18.22), VisionConstants.kAprilTags.get(6).pose.getZ(), 0.001);
        assertEquals(Units.inchesToMeters(18.22), VisionConstants.kAprilTags.get(7).pose.getZ(), 0.001);
    }

    @Test
    public void testStagedGameElements() {
        // TODO: where is the staged game piece information in our code? Not in FieldConstants.
        // Staged Game pieces are placed at
        //    X is 18 feet 8 inches from front of Grid. (Game Manual)
        //    Y
        //       ,  3 feet 0.25 inches (Game Manual)
        //       ,  7 feet 0.25 inches (Game Manual)
        //       , 11 feet 0.25 inches (Game Manual)
        //       , 15 feet 0.25 inches (Game Manual)

        // The X value for the staged pieces is difficult.
        //
        // Game Manual values
        //   The front of the middle cube node is 1 foot 2.25 inches from front of Grid (Game Manual figure 5-15)
        //   The depth of the Grid is 4 feet 8.25 inches == 56.25 inches (Game Manual figure 5-13)
        //   that puts the front of Blue middle cube node at 56.25 - 14.25 = 42.00 inches
        //   Contradiction!!! the AprilTag x-coordinate is less at 40.45 inches
        // Field Drawings are different.
        //   front of middle cube node is 14.28 inches from front of grid (close to GM).
        //   depth of grid is 54.05 inches
        //   that puts the front of Blue middle cube note at 54.05 - 14.28 = 39.77
        // CAD says 14.06 inches - 0.26 for AprilTag
        //
        // The AprilTag is mounted on the front of the middle cube node
        //   The x-coordinate for the AprilTag is 40.45 inches. That's an error of 0.68 inches.
        // The AprilTag is spaced out some amount, but I do not think it is 0.75. Maybe 0.25 between Alliance wall and Grid?
        // TODO: check AprilTag spacing from front of middle cube node.
        // spacing is apparently 0.26 inches: 0.25 for the plexiglass and 0.01 for the sticker?
        //
        // 54.05 - 14.06 = 39.99
        // add 0.26 for AprilTag = 40.25 -- that's 0.20 inches off of actual location.
        // If I add 0.19 for field margin, then I'm within 0.01 inch.
        double marginField = 0.19;
        double thickAprilTag = 0.26;
        double depthHybrid = 14.06;
        double depthGrid = 54.05;
        assertEquals(40.45, depthGrid - depthHybrid + thickAprilTag + marginField, 0.02);
     }
      
}
