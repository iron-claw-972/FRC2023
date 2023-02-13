package frc.robot.constants;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Optional;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;

/**
 * Test the ChargedUp field layout.
 */
public class ChargedUpConstantsTest {

    @Test
    public void testFieldDimensions() {
        // check length of field
        assertEquals(54.0, Units.metersToFeet(ChargedUpConstants.fieldLength), 0.001);
        // check width of field
        assertEquals(27.0, Units.metersToFeet(ChargedUpConstants.fieldWidth), 0.001);
    }

    @Test
    public void testAprilTag3() {
        // get April Tag with ID 3
        Optional<Pose3d> opose3d = ChargedUpConstants.aprilTagFieldLayout.getTagPose(3);

        // Tag 3 should exist
        assertTrue(opose3d.isPresent());

        // if April Tag 3 is present in the layout
        if (opose3d.isPresent()) {
            // get its pose
            Pose3d pose3d = opose3d.get();

            // x,y,z should be within a millimeter
            assertEquals(Units.inchesToMeters(610.77), pose3d.getX(), 0.001);
            assertEquals(Units.inchesToMeters(174.19), pose3d.getY(), 0.001);
            assertEquals(Units.inchesToMeters( 18.22), pose3d.getZ(), 0.001);

            // answer might be plus or minus pi
            assertEquals(0.0, Math.IEEEremainder(Math.PI - pose3d.getRotation().getAngle(), 2.0 * Math.PI), 0.001);
        }
    }

    @Test
    public void testAprilTag7() {
        // get April Tag with ID 7
        Optional<Pose3d> opose3d = ChargedUpConstants.aprilTagFieldLayout.getTagPose(7);

        // Tag 7 should exist
        assertTrue(opose3d.isPresent());

        // if April Tag 7 is present in the layout
        if (opose3d.isPresent()) {
            // get its pose
            Pose3d pose3d = opose3d.get();

            // x,y,z should be within a millimeter
            assertEquals(Units.inchesToMeters( 40.45), pose3d.getX(), 0.001);
            assertEquals(Units.inchesToMeters(108.19), pose3d.getY(), 0.001);
            assertEquals(Units.inchesToMeters( 18.22), pose3d.getZ(), 0.001);

            // angle should be zero
            assertEquals(0.0, Math.IEEEremainder(pose3d.getRotation().getAngle(), 2.0 * Math.PI), 0.001);
        }
    }
}
