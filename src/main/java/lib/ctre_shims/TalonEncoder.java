package lib.ctre_shims;

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;

/** A class to read encoder data from CTRE motors, frc::Encoder compatible. */
public class TalonEncoder implements Sendable, AutoCloseable {
  private final BaseTalon m_motor;
  private double m_distancePerPulse = 1;

  public TalonEncoder(BaseTalon motor) {
    this(motor, false);
  }

  public TalonEncoder(BaseTalon motor, boolean reverseDirection) {
    m_motor = motor;
    setReverseDirection(reverseDirection);
    SendableRegistry.addLW(this, "Talon Encoder", motor.getDeviceID());
  }

  @Override
  public void close() {
    SendableRegistry.remove(this);
  }

  /**
   * Gets the current count. Returns the current count on the Encoder. This method compensates for
   * the decoding type.
   *
   * @return Current count from the Encoder adjusted for the 1x, 2x, or 4x scale factor.
   */
  public int get() {
    return (int) m_motor.getSelectedSensorPosition();
  }

  /** Reset the Encoder distance to zero. Resets the current count to zero on the encoder. */
  public void reset() {
    m_motor.setSelectedSensorPosition(0);
  }

  /**
   * Returns the period of the most recent pulse. Returns the period of the most recent Encoder
   * pulse in seconds. This method compensates for the decoding type.
   *
   * <p><b>Warning:</b> This returns unscaled periods. Use getRate() for rates that are scaled using
   * the value from setDistancePerPulse().
   *
   * @return Period in seconds of the most recent pulse.
   * @deprecated Use getRate() in favor of this method.
   */
  @Deprecated
  public double getPeriod() {
    // distance / (distance / second) = seconds
    return m_distancePerPulse / getRate();
  }

  /**
   * The last direction the encoder value changed.
   *
   * @return The last direction the encoder value changed.
   */
  public boolean getDirection() {
    if (getRate() >= 0) {
      return true;
    } else {
      return false;
    }
  }

  /**
   * Get the distance the robot has driven since the last reset as scaled by the value from {@link
   * #setDistancePerPulse(double)}.
   *
   * @return The distance driven since the last reset
   */
  public double getDistance() {
    return get() * m_distancePerPulse;
  }

  /**
   * Get the current rate of the encoder. Units are distance per second as scaled by the value from
   * setDistancePerPulse().
   *
   * @return The current rate of the encoder.
   */
  public double getRate() {
    return m_motor.getSelectedSensorVelocity() * 10 * m_distancePerPulse;
  }

  /**
   * Set the distance per pulse for this encoder. This sets the multiplier used to determine the
   * distance driven based on the count value from the encoder. Do not include the decoding type in
   * this scale. The library already compensates for the decoding type. Set this value based on the
   * encoder's rated Pulses per Revolution and factor in gearing reductions following the encoder
   * shaft. This distance can be in any units you like, linear or angular.
   *
   * @param distancePerPulse The scale factor that will be used to convert pulses to useful units.
   */
  public void setDistancePerPulse(double distancePerPulse) {
    m_distancePerPulse = distancePerPulse;
  }

  /**
   * Get the distance per pulse for this encoder.
   *
   * @return The scale factor that will be used to convert pulses to useful units.
   */
  public double getDistancePerPulse() {
    return m_distancePerPulse;
  }

  /**
   * Set the direction sensing for this encoder. This sets the direction sensing on the encoder so
   * that it could count in the correct software direction regardless of the mounting.
   *
   * @param reverseDirection true if the encoder direction should be reversed
   */
  public void setReverseDirection(boolean reverseDirection) {
    m_motor.setSensorPhase(reverseDirection);
  }

  /**
   * Set the Samples to Average which specifies the number of samples of the timer to average when
   * calculating the period. Perform averaging to account for mechanical imperfections or as
   * oversampling to increase resolution.
   *
   * <p>Defaults to 64. See also, CTRE's ConfigVelocityMeasurementPeriod().
   * https://docs.ctre-phoenix.com/en/stable/ch14_MCSensor.html#changing-velocity-measurement-parameters
   * See also, the note in Measurement-Delays.md in the root of this repository.
   *
   * @param samplesToAverage The number of samples to average (one of 1, 2, 4, 8, 16, 32, or 64).
   */
  public void setSamplesToAverage(int samplesToAverage) {
    // (n & (n-1)) checks if it is a power of two. See:
    // http://
    // www.graphics.stanford.edu/~seander/bithacks.html#DetermineIfPowerOf2
    if (samplesToAverage < 1
        || samplesToAverage > 64
        || (samplesToAverage & (samplesToAverage - 1)) != 0) {
      throw new IllegalArgumentException(
          "Samples to average must be a power of 2 between 1 and 64, got " + samplesToAverage);
    }
    m_motor.configVelocityMeasurementWindow(samplesToAverage);
  }

  /**
   * Get the Samples to Average which specifies the number of samples of the timer to average when
   * calculating the period. Perform averaging to account for mechanical imperfections or as
   * oversampling to increase resolution.
   *
   * <p>Defaults to 64. See also, CTRE's ConfigVelocityMeasurementPeriod().
   * https://docs.ctre-phoenix.com/en/stable/ch14_MCSensor.html#changing-velocity-measurement-parameters
   * See also, the note in Measurement-Delays.md in the root of this repository.
   *
   * @return samplesToAverage The number of samples to average (one of 1, 2, 4, 8, 16, 32, or 64).
   */
  public int getSamplesToAverage() {
    return (int) m_motor.configGetParameter(ParamEnum.eSampleVelocityWindow, 0);
  }

  public BaseTalon getMotor() {
    return m_motor;
  }

  public FeedbackDevice getSelectedFeedbackSensor() {
    return FeedbackDevice.valueOf(m_motor.configGetParameter(ParamEnum.eFeedbackSensorType, 0));
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType(
        String.format("Encoder (%s)", getSelectedFeedbackSensor().name()));
    builder.addDoubleProperty("Speed", this::getRate, null);
    builder.addDoubleProperty("Distance", this::getDistance, null);
    builder.addDoubleProperty("Distance per Tick", this::getDistancePerPulse, null);
  }
}
