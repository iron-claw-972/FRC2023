package frc.robot.constants;

import edu.wpi.first.math.util.Units;

/**
 * Container class for arm constants.
 */
public class ArmConstants {
  
  // TODO: arm ids - TBD (fake using 5 -- 1-4 have been taken)
  public static final int kMotorId = 5;

  // TODO: PID values - TBD (fake values for now)
  public static final double kP = 1.0;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  public static final double kTolerance = Units.degreesToRadians(1);
  public static final double kMinMotorPower = -0.05;
  public static final double kMaxMotorPower = +0.05;

  // TODO: distance values - TBD (fake values for now)
  public static final double kInitialPosition = 0.0;
  public static final double kShelfPosition = 0.4;
  public static final double kIntakePosition = 0.3;
  public static final double kMiddlePosition = 0.4;
  public static final double kTopPosition = 0.5;
}
