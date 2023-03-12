package frc.robot.constants;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.swerve.DriveConstants;

public class WristConstants {
  public static final int kMotorID = 16;
  public static final boolean kEnableCurrentLimit = true;
  public static final int kContinuousCurrentLimit = 30;
  public static final int kPeakCurrentLimit = 45;
  public static final double kPeakCurrentDuration = 0.5;

  public static final double kP = 0;
  public static final double kI = 0;
  public static final double kD = 0;
  public static final double kF = 0;

  public static final double kGravityCompensation = 0.0;
  
  public static final double kTolerance = 0.1;
  public static final double kMinMotorPower = -0.3;
  public static final double kMaxMotorPower = 0.3;

  public static final TalonFXInvertType kMotorInvert = TalonFXInvertType.Clockwise;

  public static final NeutralMode kNeutralMode = NeutralMode.Brake;

  public static final int kAbsEncoderPort = 0;
  public static final double kEncoderOffset = 0;

  public static final double kStowPos = 0.96;
  public static final double kBottomNodePos = 0.49;
  public static final double kMiddleNodePos = 0.8;
  public static final double kTopNodePos = 0.712;
  public static final double kIntakeConePos = 0.49;
  public static final double kShelfPos = 0.49;
  public static final double kIntakeCubePos = 0.49;
  
  //SIM
  public static final DCMotor kGearBox = DCMotor.getFalcon500(1);
  public static final double kArmReduction = 100/1;
  public static final double kArmLength= 16.1;
  public static final double kArmMass=0;

  public static final int kMotorPort = 0;
  public static final int kEncoderAChannel = 0;
  public static final int kEncoderBChannel = 1;
  public static final int kJoystickPort = 0;

  public static final String kArmPositionKey = "ArmPosition";
  public static final String kArmPKey = "ArmP";
  //public static final DCMotor kGearBox = DCMotor.getFalcon500(1);
  public static final double kMinAngleRads = Units.degreesToRadians(-75);
  public static final double kMaxAngleRads = Units.degreesToRadians(255);
  // The P gain for the PID controller that drives this arm.
  public static final double kDefaultArmKp = 50.0;
  public static final double kDefaultArmSetpointDegrees = 75.0;

  // distance per pulse = (angle per revolution) / (pulses per revolution)
  //  = (2 * PI rads) / (4096 pulses)
  public static final double kArmEncoderDistPerPulse = 2.0 * Math.PI / 4096;

  // public static final double kArmReduction = 200;
  // public static final double kArmMass = 8.0; // Kilograms
  // public static final double kArmLength = Units.inchesToMeters(30);
  // public static final double kMinAngleRads = Units.degreesToRadians(-75);
  // public static final double kMaxAngleRads = Units.degreesToRadians(255);

  public static final double kMaxArmExtensionPos = 0.49; 

}
