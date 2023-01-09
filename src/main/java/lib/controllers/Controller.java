package lib.controllers;

import edu.wpi.first.wpilibj.Joystick;

public class Controller {
  protected final Joystick m_controller;

  public Controller(int port) {
    this.m_controller = new Joystick(port);
  }
}
