package frc.robot.util;

import static org.junit.Assert.assertEquals;

import org.junit.*;

/**
 * Example of a JUnit test class.
 * This test should run everytime someone builds the robot code.
 */
public class ArithTest {

    /**
     * Test if floating point addition works.
     */
    @Test
    public void testSimpleArith() {
        assertEquals("addition does not work", 5.0, 2.0 + 3.0,  0.0001);
    }
    
}
