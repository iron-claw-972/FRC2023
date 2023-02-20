package lib.controllers;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.*;
import java.util.function.DoubleSupplier;

public class GameController extends Controller {
  public final JoystickButton A, B, X, Y, LB, RB, BACK, START, LEFT_JOY, RIGHT_JOY;
  public final POVButton DPAD_UNPRESSED, DPAD_UP, DPAD_UP_RIGHT, DPAD_RIGHT, DPAD_DOWN_RIGHT, DPAD_DOWN, DPAD_DOWN_LEFT, DPAD_LEFT, DPAD_UP_LEFT;
  public final Trigger LEFT_TRIGGER_BUTTON, RIGHT_TRIGGER_BUTTON, ALL_UP, ALL_DOWN, ALL_LEFT, ALL_RIGHT;

  public GameController(int port) {
    super(port);

    A = new JoystickButton(m_controller, 1);
    B = new JoystickButton(m_controller, 2);
    X = new JoystickButton(m_controller, 3);
    Y = new JoystickButton(m_controller, 4);
    LB = new JoystickButton(m_controller, 5);
    RB = new JoystickButton(m_controller, 6);
    BACK = new JoystickButton(m_controller, 7);
    START = new JoystickButton(m_controller, 8);
    LEFT_JOY = new JoystickButton(m_controller, 9);
    RIGHT_JOY = new JoystickButton(m_controller, 10);

    DPAD_UNPRESSED = new POVButton(m_controller, -1);
    DPAD_UP = new POVButton(m_controller, 0);
    DPAD_UP_RIGHT = new POVButton(m_controller, 45);
    DPAD_RIGHT = new POVButton(m_controller, 90);
    DPAD_DOWN_RIGHT = new POVButton(m_controller, 135);
    DPAD_DOWN = new POVButton(m_controller, 180);
    DPAD_DOWN_LEFT = new POVButton(m_controller, 235);
    DPAD_LEFT = new POVButton(m_controller, 270);
    DPAD_UP_LEFT = new POVButton(m_controller, 315);

    LEFT_TRIGGER_BUTTON = new Trigger(() -> LEFT_TRIGGER() > 0.5);
    RIGHT_TRIGGER_BUTTON = new Trigger(() -> RIGHT_TRIGGER() > 0.5);
    ALL_UP = new Trigger(DPAD_UP.or(DPAD_UP_LEFT).or(DPAD_UP_RIGHT));
    ALL_DOWN = new Trigger(DPAD_DOWN.or(DPAD_DOWN_LEFT).or(DPAD_DOWN_RIGHT));
    ALL_LEFT = new Trigger(DPAD_LEFT.or(DPAD_UP_LEFT).or(DPAD_DOWN_LEFT));
    ALL_RIGHT = new Trigger(DPAD_RIGHT.or(DPAD_UP_RIGHT).or(DPAD_DOWN_RIGHT));
  }

  public double LEFT_X() { return m_controller.getRawAxis(0); }
  public double LEFT_Y() { return m_controller.getRawAxis(1); }
  public double LEFT_TRIGGER() { return m_controller.getRawAxis(2); }
  public double RIGHT_TRIGGER() { return m_controller.getRawAxis(3); }
  public double RIGHT_X() { return m_controller.getRawAxis(4); }
  public double RIGHT_Y() { return m_controller.getRawAxis(5); }

  public void setRumble(double leftRumble, double rightRumble) {
    m_controller.setRumble(RumbleType.kLeftRumble, leftRumble);
    m_controller.setRumble(RumbleType.kRightRumble, rightRumble);
  }
}