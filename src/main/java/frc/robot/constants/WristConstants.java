package frc.robot.constants;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;

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


  public static final double kMaxArmExtensionPos = 0.49; 

}
