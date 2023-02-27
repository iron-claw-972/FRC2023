package lib.controllers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.*;

public class Ex3DProController extends Controller {
  public Ex3DProController(int port) {
    super(port);
  }

  public enum Ex3DProButton {
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

    Ex3DProButton(final int id) {
      this.id = id;
    }
  }

  public enum Ex3DProAxis {
    X(0),
    Y(1),
    Z(2),
    SLIDER(3);

    public final int id;

    Ex3DProAxis(final int id) {
      this.id = id;
    }
  }

  public enum Ex3DProHatSwitch {
    UNPRESSED(-1),
    UP(0),
    UP_RIGHT(45),
    RIGHT(90),
    DOWN_RIGHT(135),
    DOWN(180),
    DOWN_LEFT(235),
    LEFT(270),
    UP_LEFT(315);

    public final int angle;

    Ex3DProHatSwitch(final int angle) {
      this.angle = angle;
    }
  }

  public Trigger get(Ex3DProButton button) {
    return new Trigger(() -> m_controller.getRawButton(button.id));
  }

  public double get(Ex3DProAxis axis) {
    return m_controller.getRawAxis(axis.id);
  }

  public Trigger get(Ex3DProHatSwitch hatSwitch) {
    return new Trigger(() -> m_controller.getPOV() == hatSwitch.angle);
  }

  public Joystick get() {
    return m_controller;
  }
}
