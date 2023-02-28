package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public class DeployingBarConstants {
  public static final int kMotor = 0;
  public static final double kStowRotation = 0, kDeployedRotation = Units.degreesToRadians(100);
  public static final double kP = 1, kI = 0, kD = 0;
  public static final double kTolerance = 0.01;
  public static final double kGearRatio = 1;
  public static final double kDistancePerPulse = 2 * Math.PI / kGearRatio / 2048;
}
