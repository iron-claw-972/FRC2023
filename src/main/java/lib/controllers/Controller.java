package lib.controllers;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Controller {
  protected final Joystick m_controller;

  public Controller(int port) {
    this.m_controller = new Joystick(port);
  }

  public Trigger get(BooleanSupplier sup) {
    return new Trigger(sup);
  }
}
