package lib.controllers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.*;

public class MadCatzController extends Controller {
  public final Trigger
      ALL_UP = get(MadCatzHatSwitch.UP).or(get(MadCatzHatSwitch.UP_LEFT)).or(get(MadCatzHatSwitch.UP_RIGHT)),
      ALL_DOWN = get(MadCatzHatSwitch.DOWN).or(get(MadCatzHatSwitch.DOWN_LEFT)).or(get(MadCatzHatSwitch.DOWN_RIGHT)),
      ALL_LEFT = get(MadCatzHatSwitch.LEFT).or(get(MadCatzHatSwitch.UP_LEFT)).or(get(MadCatzHatSwitch.DOWN_LEFT)),
      ALL_RIGHT = get(MadCatzHatSwitch.RIGHT).or(get(MadCatzHatSwitch.UP_RIGHT)).or(get(MadCatzHatSwitch.DOWN_RIGHT));

  public MadCatzController(int port) {
    super(port);
  }

  public enum MadCatzButton {
    B1(1),
    B2(2),
    B3(3),
    B4(4),
    B6(6),
    B7(7);

    public final int id;

    MadCatzButton(final int id) {
      this.id = id;
    }
  }

  public enum MadCatzAxis {
    X(0),
    Y(1),
    SLIDER(2),
    ZROTATE(3);

    public final int id;

    MadCatzAxis(final int id) {
      this.id = id;
    }
  }

  public enum MadCatzHatSwitch {
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

    MadCatzHatSwitch(final int angle) {
      this.angle = angle;
    }
  }

  public JoystickButton get(MadCatzButton button) {
    return new JoystickButton(m_controller, button.id);
  }

  public double get(MadCatzAxis axis) {
    return m_controller.getRawAxis(axis.id);
  }

  public POVButton get(MadCatzHatSwitch hatSwitch) {
    return new POVButton(m_controller, hatSwitch.angle);
  }

  public Trigger get(Trigger trigger) {
    return trigger;
  }

  public Joystick get() {
    return m_controller;
  }
}
