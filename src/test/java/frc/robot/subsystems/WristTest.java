package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import frc.robot.constants.Constants;
import frc.robot.constants.WristConstants;

/**
 * Test the Wrist subsystem.
 * <p>
 * The tests uncovered a problem with the DutyCycleEncoderSim class.
 */
public class WristTest {
  // get the Wrist tab (needed to construct a Wrist)
  static ShuffleboardTab m_wristTab = Shuffleboard.getTab("Wrist");

  // wrist subsystem
  Wrist m_wrist;

  @BeforeEach
  public void prepare() {
    // The kUseTelemetry must be turned off to avoid an Illegal Argument exception.
    // Otherwise there will be multiple .add method calls to the tab with the same
    // key.
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
    double deltaRadians = 2.0 * Math.PI / 1024.0;// resulution of encoder in radians

    // convert to arc error
    double arcError = deltaRadians * WristConstants.kLength;

    // Limit of the angle measurement
    System.out.printf("Wrist height resolution = %8f meters (%8f inches)\n", arcError, Units.metersToInches(arcError));
    // want the resolution to be less than 3 mm (about 0.125 inches)
    assertTrue(arcError < 0.003);

    // kTolerance is worse
    arcError = WristConstants.kTolerance * WristConstants.kLength;
    System.out.printf("Wrist height tolerance  = %8f meters (%8f inches)\n", arcError, Units.metersToInches(arcError));
    assertTrue(arcError < 0.01);
  }

  /**
   * Check the wrist torque.
   */
  @Test
  public void testWristTorque() {
    // The gearbox will give stall torque
    DCMotor gearbox = WristConstants.kGearBox;
    double torqueStall = gearbox.stallTorqueNewtonMeters;

    // the gearing will increase the available torque
    double torqueAvailable = torqueStall * WristConstants.kGearRatio;
    System.out.printf("Wrist torqueAvailable       = %8.4f Nm\n", torqueAvailable);

    // torque from current limited motor
    double torqueLimited = torqueAvailable * WristConstants.kContinuousCurrentLimit / gearbox.stallCurrentAmps;
    System.out.printf("Wrist torqueLimited         = %8.4f Nm\n", torqueLimited);
    double torqueLimited2 = WristConstants.kContinuousCurrentLimit * gearbox.KtNMPerAmp * WristConstants.kGearRatio;
    // System.out.printf(" KtNMPerAmp = %8.4f Nm\n", torqueLimited2);
    assertEquals(torqueLimited, torqueLimited2, 0.001);

    // estimated torque requirement to overcome gravity
    double torqueRequired = 0.67 * WristConstants.kMomentOfInertia / WristConstants.kLength
        * Constants.kGravitationalAccel;
    System.out.printf("Wrist torque                = %8.4f Nm\n", torqueRequired);

    // torque added per tick (for the PID P output)
    double torqueAdd = WristConstants.kP * (2.0 * Math.PI / 1024.0) * torqueAvailable;
    System.out.printf("Wrist torque step           = %8.4f Nm\n", torqueAdd);

    double ratioTorqueRequiredToAvailable = torqueRequired / torqueAvailable;
    System.out.printf("torque required / available = %8.4f PWM\n", ratioTorqueRequiredToAvailable);
    // System.out.printf("Wrist fraction amps = %8.4f\n", fraction *
    // gearbox.stallCurrentAmps);

    System.out.printf("Wrist gravity comp          = %8.4f PWM\n", WristConstants.kGravityCompensation);

    double ohms = gearbox.rOhms;
    double ampsContinuous = WristConstants.kContinuousCurrentLimit;
    double wattsContinuous = ampsContinuous * ampsContinuous * ohms;
    System.out.printf("Wrist max power             = %8.4f Watts\n", wattsContinuous);

    double amps = torqueRequired / WristConstants.kGearRatio / gearbox.KtNMPerAmp;
    double wattsRequired = amps * amps * ohms;
    System.out.printf("Wrist expected power        = %8.4f Watts\n", wattsRequired);
  }

  /**
   * Test the encoder of the wrist.
   */
  @Test
  public void testWristEncoder() {
    DutyCycleEncoder encoder = m_wrist.m_absEncoder;
    DutyCycleEncoderSim encoderSim = m_wrist.m_absEncoderSim;

    // the encoder objects should exist
    assertNotNull(encoder);
    assertNotNull(encoderSim);

    // The offset and scale were set when the Wrist was constructed ...
    assertEquals(WristConstants.kEncoderOffset, encoder.getPositionOffset(), 0.00001);
    assertEquals(-2.0 * Math.PI, encoder.getDistancePerRotation(), 0.000001);

    // the absolute encoder should start out at zero
    // These results may not be correct but they are what we get
    assertEquals(0.0, encoder.getAbsolutePosition(), 0.001);
    assertEquals(0.0, encoder.getDistance(), 0.001);

    // we can set rotations
    encoderSim.set(WristConstants.kStowPos / encoder.getDistancePerRotation());

    assertEquals(WristConstants.kStowPos, encoder.get() * encoder.getDistancePerRotation(), 0.001);

    // distance should be a multiple
    assertEquals(WristConstants.kStowPos, encoder.getDistance(), 0.001);

    encoderSim.setDistance(WristConstants.kStowPos);
    assertEquals(WristConstants.kStowPos, encoder.getDistance(), 0.001);
  }

  /**
   * Test the duty cycle encoder object.
   */
  @Test
  public void testDutyCycleEncoder() {
    int channel = 6;
    DutyCycleEncoder dceEncoder = new DutyCycleEncoder(channel);
    DutyCycleEncoderSim dceSim = new DutyCycleEncoderSim(dceEncoder);

    double gain = 3.0;
    dceEncoder.setDistancePerRotation(gain);

    double rotations = 1.0;
    dceSim.set(rotations);

    System.out.printf(" dceEncoder.get()         = %8f\n", dceEncoder.get());

    System.out.printf(" dceEncoder.getDistance() = %8f\n", dceEncoder.getDistance());
    assertEquals(rotations, dceEncoder.get(), 0.001);
    assertEquals(rotations * gain, dceEncoder.getDistance(), 0.001);

    dceSim.setDistance(rotations * gain);

    System.out.printf(" dceEncoder.get()         = %8f\n", dceEncoder.get());

    System.out.printf(" dceEncoder.getDistance() = %8f\n", dceEncoder.getDistance());
    assertEquals(rotations, dceEncoder.get(), 0.001);
    assertEquals(rotations * gain, dceEncoder.getDistance(), 0.001);

    dceEncoder.close();
  }

  /**
   * Simulate moving the wrist.
   */
  @Test
  public void testWristMovement() {
    // position does not start at the stow position.
    // Simulate to position

    // Iterate
    for (int i = 0; i < 40; i++) {
      m_wrist.periodic();
      m_wrist.simulationPeriodic();
    }

    // make sure we are at the stow position
    assertEquals(WristConstants.kStowPos, m_wrist.getAbsEncoderPos(), 0.03);

    assertTrue(m_wrist.reachedSetpoint());

    // change the setpoint
    m_wrist.setSetpoint(WristConstants.kAutoTopCone);

    // Iterate
    for (int i = 0; i < 40; i++) {
      m_wrist.periodic();
      m_wrist.simulationPeriodic();
    }

    // make sure we are at the new position
    assertEquals(WristConstants.kAutoTopCone, m_wrist.getAbsEncoderPos(), 0.03);

    assertTrue(m_wrist.reachedSetpoint());
  }
}
