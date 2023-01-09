package lib.controllers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.*;

public class Ex3DProController extends Controller {
  public Ex3DProController(int port) {
    super(port);
  }

  public enum Button {
    B1(1),
    B2(2),
    B3(3),
    B4(4),
    B6(6),
    B7(7),
    B8(8),
    B9(9),
    B10(10),
    B11(11),
    B12(12);

    public final int id;

    Button(final int id) {
      this.id = id;
    }
  }

  public enum Axis {
    X(0),
    Y(1),
    Z(2),
    SLIDER(3);

    public final int id;

    Axis(final int id) {
      this.id = id;
    }
  }

  public JoystickButton get(Button button) {
    return new JoystickButton(m_controller, button.id);
  }

  public double get(Axis axis) {
    return m_controller.getRawAxis(axis.id);
  }

  public Joystick get() {
    return m_controller;
  }
}
