package frc.robot.constants;

import com.revrobotics.CANSparkMax.IdleMode;

public final class IntakeConstants {
  // motor ports
  public static final int kLeftMotorPort = 6;
  public static final int kRightMotorPort = 11;

  // intake speeds
  public static final double kIntakePower = -0.7;
  public static final double kOuttakePower = 0.12;
  public static final double kEjectPower = 0.4;
  public static final double kSpinningPower = 0.2;

  // distance sensor thresholds
  //TODO: test accurate numbers
  public static final double kConeDistanceThreshold = 0;
  public static final double kCubeDistanceThreshold = 0;
  public static final double kCubeTimeThreshold = 0.2; // seconds

  public static final boolean kLeftMotorInvert = false;
  public static final boolean kRightMotorInvert = true;
  public static final IdleMode kLeftMotorIdleMode = IdleMode.kBrake;
  public static final IdleMode kRightMotorIdleMode = IdleMode.kBrake;
  public static final int kMotorCurrentLimit = 10;
}