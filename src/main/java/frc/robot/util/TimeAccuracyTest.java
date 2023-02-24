package frc.robot.util;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.util.WPIUtilJNI;

/**
 * Class for testing if a value is within a certain margin of error for a certain amount of time.
 */
public class TimeAccuracyTest {
  
  private BooleanSupplier m_accuracyTest;
  private double m_setpointUpdateTime;
  private double m_errorMargin;
  private double m_timeMargin;
  private boolean m_lastUseableResult = false;

  /**
   * @param actual DoubleSupplier that returns the actual value
   * @param setpoint DoubleSupplier that returns the setpoint
   * @param errorMargin margin of error for the test to be accurate
   * @param timeMargin time in seconds that the setpoint must be held for the test to be accurate
   */
  public TimeAccuracyTest(DoubleSupplier actual, DoubleSupplier setpoint, double errorMargin, double timeMargin){
    m_errorMargin = errorMargin;
    m_timeMargin = timeMargin;
    m_setpointUpdateTime = WPIUtilJNI.now() * 1e-6;
    m_accuracyTest =  new BooleanSupplier(){
      @Override
      public boolean getAsBoolean() {
        return getDoubleAccuracyTest(actual, setpoint);
      }
    };
  }

  /**
   * Determines if the test is successful.
   * @return true if the test is successful, false if not
   */
  public boolean calculate(){
    if (m_setpointUpdateTime + m_timeMargin <= WPIUtilJNI.now() * 1e-6) m_lastUseableResult = m_accuracyTest.getAsBoolean();
    return m_lastUseableResult;
  }

  /**
   * Determines if the actual value is within the error margin of the setpoint.
   * @return true if the actual value is within the error margin of the setpoint, false if not
   */
  private boolean getDoubleAccuracyTest(DoubleSupplier actual, DoubleSupplier setpoint){
    return Math.abs(actual.getAsDouble() - setpoint.getAsDouble()) <= m_errorMargin;
  }
}
