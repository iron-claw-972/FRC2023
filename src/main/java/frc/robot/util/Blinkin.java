package frc.robot.util;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Blinkin {
  
  static private CANSparkMax m_ledController;
  
  private static CANSparkMax getController() {
    if (m_ledController == null) m_ledController = new CANSparkMax(0, MotorType.kBrushless);
    return m_ledController;
  }
  
  public void setTableInput(double tableInput) {
    getController().set(tableInput);
  }
}
