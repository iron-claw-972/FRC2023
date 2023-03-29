package frc.robot.constants;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class WristConstants {
  public static final int kMotorID = 2;
  public static final boolean kEnableCurrentLimit = true;
  public static final int kContinuousCurrentLimit = 30;
  public static final int kPeakCurrentLimit = 45;
  public static final double kPeakCurrentDuration = 0.5;

  public static final double kP = 2;
  public static final double kI = 0;
  public static final double kD = 0;
  public static final double kF = 0;

  /** the gravity compensation amount as a percent of full motor power, scaled by a cosine term (this value is the max FF) */
  public static final double kGravityCompensation = 0.08;

  // TODO: this is a tolerance of 3.6 degrees (1/16 radian -> 1 inches)
  public static final double kTolerance = Units.rotationsToRadians(0.01);
  public static final double kMotorPowerClamp = 0.8;

  public static final TalonFXInvertType kMotorInvert = TalonFXInvertType.CounterClockwise;

  public static final NeutralMode kNeutralMode = NeutralMode.Brake;

  /** RoboRIO digital input port for the wrist absolute encoder */
  public static final int kAbsEncoderPort = 7;
  /** wrist absolute encoder offset (rotations). */
  public static final double kEncoderOffset = 0.704;

  public static final double kAutoMiddle = Units.rotationsToRadians(0.132);

  public static final double kAutoTop = Units.rotationsToRadians(0.070);

  public static final double kStowPos = Units.rotationsToRadians(0.304);
  public static final double kBottomNodeCubePos = Units.rotationsToRadians(0.170);
  public static final double kMiddleNodeCubePos = Units.rotationsToRadians(0.170);
  public static final double kTopNodeCubePos = Units.rotationsToRadians(0.250);

  public static final double kBottomNodeConePos = Units.rotationsToRadians(0.070);
  public static final double kMiddleNodeConePos = Units.rotationsToRadians(0.070);
  public static final double kTopNodeConePos = Units.rotationsToRadians(0.070);
  
  public static final double kIntakeConePos = Units.rotationsToRadians(0.035); // 0.025 when untensioned
  public static final double kIntakeCubePos = Units.rotationsToRadians(0.015);
  public static final double kIntakeShelfPos = Units.rotationsToRadians(0.035);

  /** Wrist position angle minimum (radians) */
  public static final double kMinPos = Units.rotationsToRadians(-0.05);
  /** Wrist position angle maximum (radians) */
  public static final double kMaxPos = kStowPos;

  //SIM
  // to know how much the arm will move with a certain power, the sim needs to know the motor, gear ratio, MOI, and length
  public static final DCMotor kGearBox = DCMotor.getFalcon500(1);
  // TODO: check wrist gear ratios
  public static final double kGearRatio = (20.0/1.0) * (62.0/34.0) * (48.0/18.0);
  /** Wrist moment of inertia represents how hard it is to angularly accelerate (ie spin) something. */
  // TODO: MOI of 24.109 is HUGE! It is equivalent to 24 kg at 1 meter! We should have less than 5 kg at 0.5 meter.
  public static final double kMomentOfInertia = 24.109 * 0.05;
  /** Length of the wrist joint (meters). Used in simulation and extimating accuracy. */
  public static final double kLength = Units.inchesToMeters(16.1);
}
