package frc.robot.util;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Blinkin {
  
  static private CANSparkMax m_ledController;
  
  private static CANSparkMax getController() {
    if (m_ledController == null) m_ledController = new CANSparkMax(0, MotorType.kBrushless);
    return m_ledController;
  }


  enum Colors{
    RainbowRainbowPalette(-0.99),
    HotPink(0.57),
    DarkRed(0.59),
    Red(0.61),
    Orange(0.65),
    Gold(0.67),
    Yellow(0.69),
    Lime(0.73),
    Green(0.77),
    Aqua(0.81),
    SkyBlue(0.83),
    DarkBlue(0.85),
    blue(0.87),
    BlueViolet(0.89),
    Violet(0.91),
    White(0.93),
    Gray(0.95),
    DarkGray(0.97),
    Black(0.99);

    public final double m_id;
    Colors(double id){
      m_id = id;
    }

    

  }
  
  public void setTableInput(double tableInput) {
    getController().set(tableInput);
  }
}
