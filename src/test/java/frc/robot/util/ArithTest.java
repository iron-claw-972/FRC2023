package frc.robot.util;

import static org.junit.Assert.assertEquals;

import org.junit.*;

public class ArithTest {

    @Test
    public void testSimpleArith() {
        assertEquals("addition does not work", 5.0, 2.0 + 3.0,  0.0001);
    }
    
}
