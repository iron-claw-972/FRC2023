package frc.robot.constants;

import edu.wpi.first.math.system.plant.DCMotor;

public class ArmConstants {
  // arm ids - TBD
  public static final int motorID = 1;

  // pid values - TBD
  public static final double kP = -1;
  public static final double kI = -1;
  public static final double kD = -1;
  public static final double kTolerance = -1;
  public static final double minMotorPower = -1;
  public static final double maxMotorPower = -1;

  // distance values - TBD
  public static final double initialPosition = 0;
  public static final double shelfPosition = -1;
  public static final double intakePosition = -1;
  public static final double middlePosiiton = -1;
  public static final double topPosition = -1;

  // simulation constants
  public static final DCMotor armSimMotor = DCMotor.getNEO(1);
  public static final double armReduction = 45;
  public static final double armMass = 2.05;
  public static final double armLength = .24;
  public static final double armMOI = .1194;
}
