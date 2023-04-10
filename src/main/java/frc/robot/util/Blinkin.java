package frc.robot.util;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Blinkin {
  
  static private Blinkin m_singleton;
  static private CANSparkMax m_ledController;

  public Blinkin() {
    m_ledController = new CANSparkMax(0, MotorType.kBrushless);
  }
  
}
