package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.fail;

import java.io.IOException;

import org.junit.jupiter.api.Test;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import frc.robot.constants.VisionConstants;

public class VisionTest {
    /**
     * Checks that the 2023ChargedUp AprilTag field layout provided by FIRST matches the one in VisionConstants.kAprilTags
     */
    @Test
    public void testAprilTagPoses() {
        try {
            AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
            for (int i = 0; i < 8; i++) {
                assertTrue(aprilTagFieldLayout.getTagPose(i + 1).get().equals(VisionConstants.kAprilTags.get(i).pose), "AprilTag " + Integer.toString(i + 1) + " doesn't match");
            }
        } catch (IOException e) {
            fail("Could not load k2023ChargedUp file from WPILib, check that GradleRIO version >= 2023.2.1");
        }
    }
}
