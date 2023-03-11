package frc.robot.constants;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;

import edu.wpi.first.math.util.Units;
import frc.robot.constants.swerve.DriveConstants;

public class WristConstants {
  public static final int kMotorID = 0;
  public static final boolean kEnableCurrentLimit = true;
  public static final int kContinuousCurrentLimit = 25;
  public static final int kPeakCurrentLimit = 40;
  public static final double kPeakCurrentDuration = 0.1;

  public static final double kP = 0;
  public static final double kI = 0;
  public static final double kD = 0;
  public static final double kF = 0;



  public static final TalonFXInvertType kMotorInvert = TalonFXInvertType.Clockwise;

  public static final NeutralMode kNeutralMode = NeutralMode.Brake;

  public static final int kAbsEncoderPort = 0;


}
