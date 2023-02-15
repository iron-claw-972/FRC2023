package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public class ElevatorConstants {
  //TODO: Set everything to meters in whole robot code
  public static final int kMotorPort = 0; 
  public static final int kTopLimitSwitchPort = 0; 
  public static final int kBottomLimitSwitchPort = 0; 
  public static final int kAbsEncoderPort = 0; 
  public static final double kP = -1; 
  public static final double kI = -1; 
  public static final double kD = -1; 
  public static final double kHeightMiddleNodeMeters = -1; //based on Arnav's calcs: 
  public static final double kHeightTopNodeMeters = -1; //based on Arnav's calcs: 
  public static final double kHeightBottomNodeMeters = -1; //based on Arnav's calcs: 
  public static final double kMaxHeight = 0; 
  public static final double kBottomHeightMeters = 0; 
  public static final double kGearRatio = 12.8571429; //TODO: turn into fraction
  public static final double kSpoolRadius = Units.inchesToMeters(0.6875); 
  public static final double kDistPerMotorEncoderTickMeters = 2.0 * Math.PI * kSpoolRadius / kGearRatio / FalconConstants.kResolution; //TODO: Check if this calculation is correct
  public static final double kMiddleNodeSetpoint = -1; 
  public static final double kTopNodeSetpoint = -1; 
  public static final double kMotorEncoderZeroingPower = -0.25; //TODO: This may need to be changed. 
  public static final double kMotorMaxHeightRecordingPower = 0.25; //TODO: This may need to be changed. 

  //GR: 12.8571429:1
  //1.375 = radius of spool(drum radius); 
}
