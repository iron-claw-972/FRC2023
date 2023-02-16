package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.apriltag.AprilTagFieldLayout;

/**
 * Example of a JUnit test class.
 * This test should run everytime someone builds the robot code.
 * See https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/unit-testing.html
 * 
 * To disable a test, annotate with Disabled
 */
public class FieldLayoutTest {
    private AprilTagFieldLayout fieldLayout;

    @BeforeEach
    public void prepare() {
        fieldLayout = Vision.getTagFieldLayout();
    }

    @AfterEach
    public void cleanup() {}

    /**
     * Test if the field layout exists
     */
    @Test
    public void testExists() {
        assertTrue(fieldLayout.getTagPose(1).isPresent());
    }

    /**
     * Test if blue April tags are to the left of red tags
     */
    @Test
    public void testBlueOnLeft() {
        assertTrue(fieldLayout.getTagPose(5).get().getX()<fieldLayout.getTagPose(1).get().getX());
    }

    /**
     * Test if April tag 2 is above April tag 1
     */
    @Test
    public void test2Above1() {
        assertTrue(fieldLayout.getTagPose(2).get().getY()>fieldLayout.getTagPose(1).get().getY());
    }
}
