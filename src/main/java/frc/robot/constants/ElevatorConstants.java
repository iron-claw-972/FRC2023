package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public class ElevatorConstants {
  //TODO: Set everything to meters in whole robot code
  public static final int kMotorPort = 14; 
  public static final int kTopLimitSwitchPort = 4; 
  public static final int kBottomLimitSwitchPort = 5; 
  public static final int kAbsEncoderPort = 6; 
  public static final double kP = 0.1; 
  public static final double kI = 0.0; 
  public static final double kD = 0.0; 
  public static final double kHeightZeroing = -0.1; 
  public static final double kHeightBottomNode = 0.2; //based on Arnav's calcs: 
  public static final double kHeightMiddleNode = 0.3; //based on Arnav's calcs: 
  public static final double kHeightTopNode = 0.4; //based on Arnav's calcs: 
  public static final double kMaxHeight = 0.5; 
  public static final double kBottomHeight = 0.0; 
  public static final double kGearRatio = 90.0/7.0; //TODO: put in actual GR
  public static final double kSpoolRadius = Units.inchesToMeters(0.6875); 
  public static final double kDistPerMotorEncoderTick = 2.0 * Math.PI * kSpoolRadius / kGearRatio / FalconConstants.kResolution; //TODO: Check if this calculation is correct
  public static final double kMotorEncoderZeroingPower = -0.25; //TODO: This may need to be changed. 
  public static final double kMotorMaxHeightRecordingPower = 0.25; //TODO: This may need to be changed.  
  /** Normal full power level for the elevator motor. */
  public static final double kMotorLimit = 0.30;
  /** Power level to use when zeroing the encoder using the bottom limit switch */
  public static final double kMotorZeroingLimit = 0.25; 

}
