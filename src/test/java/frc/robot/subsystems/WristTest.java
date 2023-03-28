package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import frc.robot.constants.Constants;
import frc.robot.constants.WristConstants;

public class WristTest {
    // get the Wrist tab (needed to construct a Wrist)
    static ShuffleboardTab m_wristTab = Shuffleboard.getTab("Wrist");

    // wrist subsystem
    Wrist m_wrist;

    @BeforeEach
    public void prepare() {
        // This must be turned off to avoid an Illegal Argument exception.
        // Otherwise there will be multiple adds to the tab with the same key.
        Constants.kUseTelemetry = false;

        // build the wrist
        m_wrist = new Wrist(m_wristTab);
    }

    @AfterEach
    public void cleanup() {
        // deallocate resources
        m_wrist.close();
    }

    /**
     * Test the controllability of the wrist.
     */
    @Test
    public void testWristAccuracy() {
        // angular accuracy of the encoder
        double deltaRadians = 2.0 * Math.PI / 1024.0;
        // convert to arc error
        double arcError = deltaRadians * WristConstants.kLength;

        // System.out.printf("Wrist height accuracy = %8f meters (%8f inches)\n", arcError, Units.metersToInches(arcError));

        // want accuracy to be less than 3 mm (about 0.125 inches)
        assertTrue(arcError < 0.003);
    }

    /**
     * Test the encoder.
     * <p>
     * The results are unexpected: set(rotations) works but setDistance(distance) does not.
     */
    @Test
    public void testWristEncoder() {
        DutyCycleEncoder encoder = m_wrist.m_absEncoder;
        DutyCycleEncoderSim encoderSim = m_wrist.m_absEncoderSim;

        // the encoder objects should exist
        assertNotNull(encoder);
        assertNotNull(encoderSim);

        // The offset and scale should have been set...

        // 0.704
        // System.out.println(encoder.getPositionOffset());
        assertEquals(WristConstants.kEncoderOffset, encoder.getPositionOffset(), 0.00001);
        
        // -6.28318...
        // System.out.println(encoder.getDistancePerRotation());
        assertEquals(-2.0 * Math.PI, encoder.getDistancePerRotation(), 0.000001);

        // the absolution encoder should start out at zero
        assertEquals(0.0, encoder.getAbsolutePosition(), 0.001);
        assertEquals(0.0, encoder.getDistance(), 0.001);

        // now things are crazy...
        System.out.println("crazy times");

        // print the known value
        // 1.910
        System.out.println(WristConstants.kStowPos);


        // we can set rotations
        encoderSim.set(WristConstants.kStowPos);
        // 1.910
        System.out.println(encoder.get());
        assertEquals(WristConstants.kStowPos, encoder.get(), 0.001);
        // -12.0014
        System.out.println(encoder.getDistance());
        // distance should be a multiple
        assertEquals(WristConstants.kStowPos, encoder.getDistance() / encoder.getDistancePerRotation(), 0.001);

        System.out.println("Try setDistance() -- fails!");
        // we cannot set a distance
        encoderSim.setDistance(WristConstants.kStowPos);
        System.out.println(encoder.get());
        System.out.println(encoder.getDistance());
        // TODO: setDistance() fails; set() and setDistance() are the same? Only getDistance() is scaled?
        // .see https://github.com/wpilibsuite/allwpilib/blob/main/wpilibj/src/main/java/edu/wpi/first/wpilibj/simulation/DutyCycleEncoderSim.java
        // assertEquals(WristConstants.kStowPos, encoder.getDistance(), 0.001);

        // GetAbsolutePosition() - GetPositionOffset() will give an encoder absolute position relative to the last reset. 

        // System.out.println(encoder.getAbsolutePosition() - encoder.getPositionOffset());
        // System.out.println(m_wrist.getAbsEncoderPos());

        // WristConstants.kMinPos;
        // WristConstants.kMaxPos;
        // WristConstants.kEncoderOffset
    }

    /**
     * The second test.
     */
    @Test
    public void testWristMovement() {
        // position does not start at the stow position.
        // Simulate to position

        // Iterate
        for (int i = 0; i < 40; i++) {
            m_wrist.periodic();
            m_wrist.simulationPeriodic();
            // System.out.printf("goal %8f position %8f motor %8f\n", m_wrist.m_pid.getSetpoint(), m_wrist.getAbsEncoderPos(), m_wrist.m_pidPower);
        }

        // make sure we are at the stow position
        assertEquals(WristConstants.kStowPos, m_wrist.getAbsEncoderPos(), 0.03);

        // change the setpoint
        m_wrist.setSetpoint(WristConstants.kAutoTop);

        // Iterate
        for (int i = 0; i < 40; i++) {
            m_wrist.periodic();
            m_wrist.simulationPeriodic();
            // System.out.printf("goal %8f position %8f motor %8f\n", m_wrist.m_pid.getSetpoint(), m_wrist.getAbsEncoderPos(), m_wrist.m_pidPower);
        }

        // make sure we are at the stow position
        assertEquals(WristConstants.kAutoTop, m_wrist.getAbsEncoderPos(), 0.03);
    }
    
}
