package frc.robot.constants;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.util.Units;

public class BarConstants {
  public static final int kMotorID = 50; // FIXME
  public static final boolean kMotorInvert = false;
  public static final IdleMode kMotorIdleMode = IdleMode.kBrake;
  public static final double kGearRatio = (25.0 / 1.0) * (32.0 / 18.0); // 44.44:1

  public static final double kMotorEncoderDistancePerRotation = -1; // FIXME

  public static final double kP = 0.1;
  public static final double kI = 0;
  public static final double kD = 0;

  public static final double kGravityCompensationFactor = 0; // volts

  public static final int kMotorCurrentLimit = 12;

  public static final double kAngleTolerance = 0.5;
  public static final double kVelocityTolerance = 0.5;

  public static final double kStowAngle = 90;
  public static final double kDeployAngle = 0;

  public static final double kMaxAngularVelocity = 20;
  public static final double kMaxAngularAccel = 15;
}
