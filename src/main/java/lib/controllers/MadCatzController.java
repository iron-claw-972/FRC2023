package lib.controllers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.*;

public class MadCatzController extends Controller {
  public final Trigger
      ALL_UP = get(HatSwitch.UP).or(get(HatSwitch.UP_LEFT)).or(get(HatSwitch.UP_RIGHT)),
      ALL_DOWN = get(HatSwitch.DOWN).or(get(HatSwitch.DOWN_LEFT)).or(get(HatSwitch.DOWN_RIGHT)),
      ALL_LEFT = get(HatSwitch.LEFT).or(get(HatSwitch.UP_LEFT)).or(get(HatSwitch.DOWN_LEFT)),
      ALL_RIGHT = get(HatSwitch.RIGHT).or(get(HatSwitch.UP_RIGHT)).or(get(HatSwitch.DOWN_RIGHT));

  public MadCatzController(int port) {
    super(port);
  }

  public enum Button {
    B1(1),
    B2(2),
    B3(3),
    B4(4),
    B6(6),
    B7(7);

    public final int id;

    Button(final int id) {
      this.id = id;
    }
  }

  public enum Axis {
    X(0),
    Y(1),
    ZAXIS(2),
    ZROTATE(3);

    public final int id;

    Axis(final int id) {
      this.id = id;
    }
  }

  public enum HatSwitch {
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

    HatSwitch(final int angle) {
      this.angle = angle;
    }
  }

  public JoystickButton get(Button button) {
    return new JoystickButton(m_controller, button.id);
  }

  public double get(Axis axis) {
    return m_controller.getRawAxis(axis.id);
  }

  public POVButton get(HatSwitch hatSwitch) {
    return new POVButton(m_controller, hatSwitch.angle);
  }

  public Trigger get(Trigger trigger) {
    return trigger;
  }

  public Joystick get() {
    return m_controller;
  }
}
