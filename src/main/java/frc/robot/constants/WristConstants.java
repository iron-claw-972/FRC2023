package frc.robot.constants;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

public class WristConstants {
  public static final int kMotorID = 2;
  public static final boolean kEnableCurrentLimit = true;
  public static final int kContinuousCurrentLimit = 30;
  public static final int kPeakCurrentLimit = 45;
  public static final double kPeakCurrentDuration = 0.5;

  public static final double kP = 12;
  public static final double kI = 0;
  public static final double kD = RobotBase.isSimulation()? 1:0.1;
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

  public static final double kStowPos = Units.rotationsToRadians(0.304);
  public static final double kBottomNodeCubePos = Units.rotationsToRadians(0.170);
  public static final double kMiddleNodeCubePos = Units.rotationsToRadians(0.170);
  public static final double kTopNodeCubePos = Units.rotationsToRadians(0.250);
  public static final double kAutoMiddle = 0.132;

  public static final double kAutoTop = 0.070;

  public static final double kBottomNodeConePos = Units.rotationsToRadians(0.070);
  public static final double kMiddleNodeConePos = Units.rotationsToRadians(0.070);
  public static final double kTopNodeConePos = Units.rotationsToRadians(0.070);
  
  public static final double kIntakeConePos = Units.rotationsToRadians(0.025);
  public static final double kIntakeCubePos = Units.rotationsToRadians(0.015);
  public static final double kIntakeShelfPos = 0.035;

  /** Wrist position angle minimum (rotations) */
  public static final double kMinPos = -0.05;
  /** Wrist position angle maximum (rotations) */
  public static final double kMaxPos = kStowPos;

  
  //SIM
  public static final double kArmReduction = 20/1*62/34*48/18;
  public static final double kArmLength= 16.1;
  public static final double kArmMass=0;
  public static final double kMinAngleRads = 0;
  public static final double kMaxAngleRads = kStowPos;//0.96
  public static final double kMOI = 24.109;

  public static final int kMotorPort = 0;
  public static final int kEncoderAChannel = 0;
  public static final int kEncoderBChannel = 1;
  public static final int kJoystickPort = 0;

  public static final String kArmPositionKey = "ArmPosition";
  public static final String kArmPKey = "ArmP";
  public static final DCMotor kGearBox = DCMotor.getFalcon500(1);

  // The P gain for the PID controller that drives this arm.
  public static final double kDefaultArmKp = 50.0;
  public static final double kDefaultArmSetpointDegrees = 75.0;

  // distance per pulse = (angle per revolution) / (pulses per revolution)
  //  = (2 * PI rads) / (4096 pulses)
  public static final double kArmEncoderDistPerPulse = 2.0 * Math.PI / 4096;
  public static final double kMaxArmExtensionPos = 0.49;
  public static final double kGearRatio = 100/1;
  public static final double kLength = 16.1;
  // moment of inertia represents how hard it is to angularly accelerate (ie spin) something
  public static final double kMomentOfInertia = 24.109;
}
