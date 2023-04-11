package lib.controllers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.*;
import java.util.function.BooleanSupplier;

public class PS5Controller extends Controller {
  // These are the different controller triggers
  public final Trigger ALL_UP = get(DPad.UP).or(get(DPad.UP_LEFT)).or(get(DPad.UP_RIGHT)),
      ALL_DOWN = get(DPad.DOWN).or(get(DPad.DOWN_LEFT)).or(get(DPad.DOWN_RIGHT)),
      ALL_LEFT = get(DPad.LEFT).or(get(DPad.UP_LEFT)).or(get(DPad.DOWN_LEFT)),
      ALL_RIGHT = get(DPad.RIGHT).or(get(DPad.UP_RIGHT)).or(get(DPad.DOWN_RIGHT));
    
  public final BooleanSupplier 
    LEFT_STICK_LEFT = () -> get(PS5Axis.LEFT_X) < -0.75,
    LEFT_STICK_RIGHT = () -> get(PS5Axis.LEFT_X) > 0.75,
    LEFT_STICK_UP = () -> get(PS5Axis.LEFT_Y) < -0.75,
    LEFT_STICK_DOWN = () -> get(PS5Axis.LEFT_Y) > 0.75;
  public final BooleanSupplier 
    RIGHT_STICK_LEFT = () -> get(PS5Axis.RIGHT_X) < -0.75,
    RIGHT_STICK_RIGHT = () -> get(PS5Axis.RIGHT_X) > 0.75,
    RIGHT_STICK_UP = () -> get(PS5Axis.RIGHT_Y) < -0.75,
    RIGHT_STICK_DOWN = () -> get(PS5Axis.RIGHT_Y) > 0.75;

  public PS5Controller(int port) {
    super(port);
  }

  public enum PS5Button {
    SQUARE(1),
    CROSS(2),
    CIRCLE(3),
    TRIANGLE(4),
    LB(5),
    RB(6),
    LEFT_TRIGGER(7),
    RIGHT_TRIGGER(8),
    CREATE(9),
    OPTIONS(10),
    LEFT_JOY(11),
    RIGHT_JOY(12),
    PS(13),
    TOUCHPAD(14),
    MUTE(15);

    public final int id;

    PS5Button(final int id) {
      this.id = id;
    }
  }

  public enum PS5Axis {
    LEFT_X(0),
    LEFT_Y(1),
    RIGHT_X(2),
    /** note: ps5 controller trigger goes from -1 when unpressed, to 1 when fully pressed */
    LEFT_TRIGGER(3),
    /** note: ps5 controller trigger goes from -1 when unpressed, to 1 when fully pressed */
    RIGHT_TRIGGER(4),
    RIGHT_Y(5);

    public final int id;

    PS5Axis(final int id) {
      this.id = id;
    }
  }

  public enum DPad {
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

    DPad(final int angle) {
      this.angle = angle;
    }
  }

  public Trigger get(PS5Button button) {
    return new Trigger(() -> m_controller.getRawButton(button.id));
  }

  public Trigger get(BooleanSupplier booleanSupplier) {
    return new Trigger(booleanSupplier);
  }

  public double get(PS5Axis axis) {
    return m_controller.getRawAxis(axis.id);
  }

  public Trigger get(DPad dPad) {
    return new Trigger(() -> m_controller.getPOV() == dPad.angle);
  }

  public Joystick get() {
    return m_controller;
  }
}