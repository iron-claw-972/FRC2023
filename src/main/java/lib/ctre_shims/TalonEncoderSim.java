package lib.ctre_shims;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

/** Class to control a simulated encoder. */
public class TalonEncoderSim {
  private TalonEncoder m_encoder;

  private TalonSRXSimCollection m_simCollectionSRX = null;
  private TalonFXSimCollection m_simCollectionFX = null;

  /**
   * Constructs from an Encoder object.
   *
   * @param encoder Encoder to simulate
   */
  public TalonEncoderSim(TalonEncoder encoder) {
    m_encoder = encoder;

    if (m_encoder.getMotor() instanceof TalonSRX) {
      m_simCollectionSRX = new TalonSRXSimCollection(m_encoder.getMotor());
    } else if (m_encoder.getMotor() instanceof TalonFX) {
      m_simCollectionFX = new TalonFXSimCollection(m_encoder.getMotor());
    } else {
      throw new IllegalStateException(
          "Motor type " + m_encoder.getMotor().getClass().getName() + " is unsupported.");
    }
  }

  /**
   * Read the count of the encoder.
   *
   * @return the count
   */
  public int getCount() {
    return m_encoder.get();
  }

  /**
   * Change the count of the encoder.
   *
   * @param count the new count
   */
  public void setCount(int count) {
    if (m_simCollectionSRX != null) {
      FeedbackDevice selected = m_encoder.getSelectedFeedbackSensor();

      switch (selected) {
        case QuadEncoder:
        case CTRE_MagEncoder_Relative:
          m_simCollectionSRX.setQuadratureRawPosition(count);
          break;

        case Analog:
          m_simCollectionSRX.setAnalogPosition(count);
          break;

        default:
          throw new IllegalStateException(
              "Selected feedback sensor is not supported: " + selected.name());
      }
    } else if (m_simCollectionFX != null) {
      m_simCollectionFX.setIntegratedSensorRawPosition(count);
    } else {
      // This should have errored already in the constructor.
      assert false;
    }
  }

  /**
   * Read the period of the encoder.
   *
   * @return the encoder period
   */
  public double getPeriod() {
    return m_encoder.getRate();
  }

  /**
   * Change the encoder period.
   *
   * @param period the new period
   */
  public void setPeriod(double period) {
    // seconds -> distance/second
    // m_distancePerPulse (distance) / period (seconds) = distance / second
    setRate(m_encoder.getDistancePerPulse() / period);
  }

  // These are no-op in WPILib
  // boolean getReset();
  // void setReset();

  /**
   * Get the direction of the encoder.
   *
   * @return the direction of the encoder
   */
  public boolean getDirection() {
    return m_encoder.getDirection();
  }

  /**
   * Get the samples-to-average value.
   *
   * <p>See {@link TalonEncoder#getSamplesToAverage()}.
   *
   * @return the samples-to-average value
   */
  public int getSamplesToAverage() {
    return m_encoder.getSamplesToAverage();
  }

  /**
   * Set the samples-to-average value.
   *
   * <p>See {@link TalonEncoder#setSamplesToAverage()}.
   *
   * @param samplesToAverage the new value
   */
  public void setSamplesToAverage(int samplesToAverage) {
    m_encoder.setSamplesToAverage(samplesToAverage);
  }

  /**
   * Change the encoder distance.
   *
   * @param distance the new distance
   */
  public void setDistance(double distance) {
    setCount((int) (distance / m_encoder.getDistancePerPulse()));
  }

  /**
   * Read the distance of the encoder.
   *
   * @return the encoder distance
   */
  public double getDistance() {
    return m_encoder.getDistance();
  }

  /**
   * Change the rate of the encoder.
   *
   * @param rate the new rate
   */
  public void setRate(double rate) {
    int rateInNativeUnits = (int) (rate / (10 * m_encoder.getDistancePerPulse()));

    if (m_simCollectionSRX != null) {
      FeedbackDevice selected = m_encoder.getSelectedFeedbackSensor();

      switch (selected) {
        case QuadEncoder:
        case CTRE_MagEncoder_Relative:
          m_simCollectionSRX.setQuadratureVelocity(rateInNativeUnits);
          break;

        case Analog:
          m_simCollectionSRX.setAnalogVelocity(rateInNativeUnits);
          break;

        default:
          throw new IllegalStateException(
              "Selected feedback sensor is not supported: " + selected.name());
      }
    } else if (m_simCollectionFX != null) {
      m_simCollectionFX.setIntegratedSensorVelocity(rateInNativeUnits);
    } else {
      // This should have errored already in the constructor.
      assert false;
    }
  }

  /**
   * Get the rate of the encoder.
   *
   * @return the rate of change
   */
  public double getRate() {
    return m_encoder.getRate();
  }

  /** Resets all simulation data for this encoder. */
  public void resetData() {
    setDistance(0);
    setRate(0);
  }
}
