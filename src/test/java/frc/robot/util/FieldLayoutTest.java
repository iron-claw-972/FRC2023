package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.fail;

import java.io.IOException;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.VisionConstants;

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
    fieldLayout = new Vision(Shuffleboard.getTab("Vision"), VisionConstants.kCameras).getAprilTagFieldLayout();
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

  /**
   * Checks that the 2023ChargedUp AprilTag field layout provided by FIRST matches the one in VisionConstants.kAprilTags
   */
  @Test
  public void testAprilTagPoses() {
    try {
      AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
      for (int i = 0; i < 8; i++) {
        assertTrue(aprilTagFieldLayout.getTagPose(i + 1).get().equals(FieldConstants.kAprilTags.get(i).pose), "AprilTag " + Integer.toString(i + 1) + " doesn't match");
      }
    } catch (IOException e) {
      fail("Could not load k2023ChargedUp file from WPILib, check that GradleRIO version >= 2023.2.1");
    }
  }
}
