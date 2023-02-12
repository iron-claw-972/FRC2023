package frc.robot.util;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.util.WPIUtilJNI;

public class TimeAccuracyTest {
  
  DoubleSupplier m_actual, m_setpoint;
  BooleanSupplier m_accuracyTest;
  double m_recentSetpoint;
  double m_setpointUpdateTime;
  double m_errorMargin;
  double m_timeMargin;
  boolean m_lastUseableResult = false;

  public TimeAccuracyTest(BooleanSupplier accuracyTest, DoubleSupplier setpoint, double timeMargin){
    m_setpoint = setpoint;
    m_timeMargin = timeMargin;
    m_recentSetpoint = m_setpoint.getAsDouble();
    m_setpointUpdateTime = WPIUtilJNI.now() * 1e-6;
    m_accuracyTest =  accuracyTest;
  }

  public TimeAccuracyTest(DoubleSupplier actual, DoubleSupplier setpoint, double errorMargin, double timeMargin){
    m_actual = actual;
    m_setpoint = setpoint;
    m_errorMargin = errorMargin;
    m_timeMargin = timeMargin;
    m_recentSetpoint = m_setpoint.getAsDouble();
    m_setpointUpdateTime = WPIUtilJNI.now() * 1e-6;
    m_accuracyTest =  this::getDoubleAccuracyTest;
  }

  public boolean calculate(){
    if (m_setpointUpdateTime + m_timeMargin <= WPIUtilJNI.now() * 1e-6) m_lastUseableResult = m_accuracyTest.getAsBoolean();
    return m_lastUseableResult;
  }

  private boolean getDoubleAccuracyTest(){
    return Math.abs(m_actual.getAsDouble() - m_setpoint.getAsDouble()) <= m_errorMargin;
  }
}
