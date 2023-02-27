package lib.controllers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class PistolController extends Controller {
  public final Trigger TOP_BACK_ONLY = get(Button.TOP_BACK).and(get(Button.TOP_FRONT).negate()),
      TOP_FRONT_ONLY = get(Button.TOP_FRONT).and(get(Button.TOP_BACK).negate()),
      BOTTOM_BACK_ONLY = get(Button.BOTTOM_BACK).and(get(Button.BOTTOM_FRONT).negate()),
      BOTTOM_FRONT_ONLY = get(Button.BOTTOM_FRONT).and(get(Button.BOTTOM_BACK).negate());

  public PistolController(int port) {
    super(port);
  }

  public enum Button {
    TOP_BACK(1),
    TOP_FRONT(2),
    BOTTOM_FRONT(3),
    BOTTOM_BACK(4),
    BOTTOM(5);

    public final int id;

    Button(final int id) {
      this.id = id;
    }
  }

  public enum Axis {
    WHEEL(0),
    TRIGGER(1);

    public final int id;

    Axis(final int id) {
      this.id = id;
    }
  }

  public Trigger get(Button button) {
    return new Trigger(() -> m_controller.getRawButton(button.id));
  }

  public double get(Axis axis) {
    return m_controller.getRawAxis(axis.id);
  }
  
  public Joystick get() {
    return m_controller;
  }
}
