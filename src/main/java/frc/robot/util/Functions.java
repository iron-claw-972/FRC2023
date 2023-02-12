package frc.robot.util;

import frc.robot.constants.OIConstants;

/**
 * Utility class for useful functions.
 */
public class Functions {

  /**
   * Deadbands an input to [-1, -deadband], [deadband, 1], rescaling inputs to be linear from
   * (deadband, 0) to (1,1)
   *
   * @param input The input value to rescale
   * @param deadband The deadband
   * @return the input rescaled and to fit [-1, -deadband], [deadband, 1]
   */
  public static double deadband(double input, double deadband) {
    if (Math.abs(input) <= deadband) {
      return 0;
    } else if (Math.abs(input) == 1) {
      return input;
    } else {
      return (1 / (1 - deadband) * (input + Math.signum(-input) * deadband));
    }
  }

  /**
   * Deadbands an input to [-1, -OIConstants.kDeadband], [OIConstants.kDeadband, 1], rescaling inputs to be linear from
   * (OIConstants.kDeadband, 0) to (1,1)
   *
   * @param input The input value to rescale
   * @return the input rescaled and to fit [-1, -kDeadband], [kDeadband, 1]
   */
  public static double deadband(double input) {
    return deadband(input, OIConstants.kDeadband);
  }

  public static double simpleDeadband(double input, double deadband) {
    if (Math.abs(input) <= deadband) {
      return 0;
    } else {
      return input;
    }
  }

  /**
   * An exponential function that maintains positive or negative.
   * @param exponent the power to raise the base to
   * @param base the base which will be raised to the power
   * @return base to the power of exponent, maintaining sign of base
   */
  public static double expoMS(double base, double exponent) {
    // weird stuff will happen if you don't put a number > 0
    double finVal = Math.pow(Math.abs(base), exponent);
    if (base < 0) {
      finVal *= -1;
    }
    return finVal;
  }
  
  public static double calculateAngle(double x, double y) {
    if (x > 0) return Math.atan(y/x);
    if (x < 0) return Math.atan(y/x) + Math.PI;
    return Math.signum(y)* Math.PI/2;
  }

  public static double calculateHypotenuse(double x, double y) {
    return Math.pow(Math.pow(x,2)+Math.pow(y,2), 0.5);
  }
  
}