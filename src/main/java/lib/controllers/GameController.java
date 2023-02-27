package lib.controllers;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.*;
import java.util.function.BooleanSupplier;

public class GameController extends Controller {
  // These are the different controller triggers
  public final BooleanSupplier LEFT_TRIGGER_BUTTON = () -> get(Axis.LEFT_TRIGGER) > 0.5,
      RIGHT_TRIGGER_BUTTON = () -> get(Axis.RIGHT_TRIGGER) > 0.5;
  public final Trigger ALL_UP = get(DPad.UP).or(get(DPad.UP_LEFT)).or(get(DPad.UP_RIGHT)),
      ALL_DOWN = get(DPad.DOWN).or(get(DPad.DOWN_LEFT)).or(get(DPad.DOWN_RIGHT)),
      ALL_LEFT = get(DPad.LEFT).or(get(DPad.UP_LEFT)).or(get(DPad.DOWN_LEFT)),
      ALL_RIGHT = get(DPad.RIGHT).or(get(DPad.UP_RIGHT)).or(get(DPad.DOWN_RIGHT));

  public GameController(int port) {
    super(port);
  }

  public enum Button {
    A(1),
    B(2),
    X(3),
    Y(4),
    LB(5),
    RB(6),
    BACK(7),
    START(8),
    LEFT_JOY(9),
    RIGHT_JOY(10);

    public final int id;

    Button(final int id) {
      this.id = id;
    }
  }

  public enum Axis {
    LEFT_X(0),
    LEFT_Y(1),
    LEFT_TRIGGER(2),
    RIGHT_TRIGGER(3),
    RIGHT_X(4),
    RIGHT_Y(5);

    public final int id;

    Axis(final int id) {
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

  public enum RumbleStatus {
    RUMBLE_ON(0.7),
    RUMBLE_OFF(0);

    public final double rumbleValue;

    RumbleStatus(final double rumbleValue) {
      this.rumbleValue = rumbleValue;
    }
  }

  public Trigger get(Button button) {
    return new Trigger(() -> m_controller.getRawButton(button.id));
  }

  public double get(Axis axis) {
    return m_controller.getRawAxis(axis.id);
  }

  public Trigger get(DPad dPad) {
    return new Trigger(() -> m_controller.getPOV() == dPad.angle);
  }

  public Trigger get(BooleanSupplier condition) {
    return new Trigger(condition);
  }

  public Trigger get(Trigger trigger) {
    return trigger;
  }

  public Joystick get() {
    return m_controller;
  }

  public void setRumble(RumbleStatus rumbleStatus) {
    m_controller.setRumble(RumbleType.kLeftRumble, rumbleStatus.rumbleValue);
    m_controller.setRumble(RumbleType.kRightRumble, rumbleStatus.rumbleValue);
  }
}