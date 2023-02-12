package frc.robot.util;

import java.util.LinkedList;
import java.util.List;

/**
 * A class for storing and processing feedforward characterization data. Used in automatic feedforward characterization.
 * @see frc.robot.commands.test.DriveFeedForwardCharacterization
 * @see frc.robot.commands.test.SteerFeedForwardCharacterizationSingle
 */
public class FeedForwardCharacterizationData {
  private PolynomialRegression m_regression;
  private final List<Double> velocityData = new LinkedList<>();
  private final List<Double> voltageData = new LinkedList<>();
  
  /**
   * Adds a data point to the data set.
   * @param velocity the velocity of the motor
   * @param voltage the voltage applied to the motor
   */
  public void add(double velocity, double voltage) {
    if (Math.abs(velocity) > 1E-4) {
      velocityData.add(Math.abs(velocity));
      voltageData.add(Math.abs(voltage));
    }
  }
  
  /**
   * Processes the data set using {@link PolynomialRegression}
   * @see PolynomialRegression
   */
  public void process() {
    m_regression = new PolynomialRegression(
      velocityData.stream().mapToDouble(Double::doubleValue).toArray(),
      voltageData.stream().mapToDouble(Double::doubleValue).toArray(),
      1
    );
  }

  /**
   * Gets the static voltage of the motor.
   * @return the static voltage of the motor
   */
  public double getStatic() {
    return m_regression.beta(0);
  }

  /**
   * Gets the velocity of the motor.
   * @return the velocity of the motor
   */
  public double getVelocity() {
    return m_regression.beta(1);
  }
  
  /**
   * Gets the variance of the data set.
   * @return the variance of the data set
   */
  public double getVariance() {
    return m_regression.R2();
  }
}

