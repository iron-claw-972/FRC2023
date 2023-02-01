package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

/**
 * Example of a JUnit test class.
 * This test should run everytime someone builds the robot code.
 * See https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/unit-testing.html
 * 
 * To disable a test, annotate with Disabled
 */
public class ArithTest {

    @BeforeEach
    public void prepare() {}

    @AfterEach
    public void cleanup() {}

    /**
     * Test if floating point addition works.
     */
    @Test
    public void testSimpleArith() {
        assertEquals(5.0, 2.0 + 3.0,  0.0001);
    }

    /**
     * Here is a disabled test
     */
    @Disabled
    @Test
    public void testNaught() {
        assertEquals(0, 0);
    }
    
}
