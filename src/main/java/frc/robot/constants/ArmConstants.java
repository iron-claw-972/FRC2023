package frc.robot.constants;

import edu.wpi.first.math.system.plant.DCMotor;

public class ArmConstants {
  // arm ids - TBD
  public final int motorID = -1;

  // pid values - TBD
  public final double kP = -1;
  public final double kI = -1;
  public final double kD = -1;

  // distance values - TBD
  public final double initialPosition = 0;
  public final double shelfPosition = -1;
  public final double intakePosition = -1;
  public final double middlePosiiton = -1;
  public final double topPosition = -1;

  // simulation constants
  public final DCMotor armSimMotor = DCMotor.getNEO(1);
  public final double armReduction = 45;
  public final double armMass = 2.05;
  public final double armLength = .24;
  public final double armMOI = .1194;
}
