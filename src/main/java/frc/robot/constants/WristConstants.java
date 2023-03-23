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

  public static final double kP = 12;
  public static final double kI = 0;
  public static final double kD = 0;
  public static final double kF = 0;

  public static final double kGravityCompensation = 0.08;

  public static final double kTolerance = 0.01;
  public static final double kMotorPowerClamp = 0.8;

  public static final TalonFXInvertType kMotorInvert = TalonFXInvertType.CounterClockwise;

  public static final NeutralMode kNeutralMode = NeutralMode.Brake;

  /** RoboRIO digital input port for the wrist absolute encoder */
  public static final int kAbsEncoderPort = 7;
  /** wrist absolute encoder offset (rotations). */
  public static final double kEncoderOffset = 0.704;

  public static final double kAutoMiddle = 0.132;

  public static final double kAutoTop = 0.070;

  public static final double kStowPos = 0.304;
  public static final double kBottomNodeCubePos = 0.170;
  public static final double kMiddleNodeCubePos = 0.170;
  public static final double kTopNodeCubePos = 0.250;

  public static final double kBottomNodeConePos = 0.070;
  public static final double kMiddleNodeConePos = 0.070;
  public static final double kTopNodeConePos = 0.070;
  
  public static final double kIntakeConePos = 0.035; // 0.025 when untensioned
  public static final double kIntakeCubePos = 0.015;
  public static final double kIntakeShelfPos = 0.035;

  /** Wrist position angle minimum (rotations) */
  public static final double kMinPos = -0.05;
  /** Wrist position angle maximum (rotations) */
  public static final double kMaxPos = kStowPos;
  
  //SIM
  public static final DCMotor kGearBox = DCMotor.getFalcon500(1);
  public static final double kGearRatio = 100/1;
  public static final double kLength = 16.1;
  // moment of inertia represents how hard it is to angularly accelerate (ie spin) something
  public static final double kMomentOfInertia = 24.109;

  public static final double kMinAngleRads = Units.rotationsToRadians(kMinPos);
  public static final double kMaxAngleRads = Units.rotationsToRadians(kMaxPos);
}
