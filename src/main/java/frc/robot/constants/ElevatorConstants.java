package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public class ElevatorConstants {
  //TODO: Set everything to meters in whole robot code
  public static final int kElevatorMotor = -1; 
  public static final double kElevatorP = -1; 
  public static final double kElevatorI = -1; 
  public static final double kElevatorD = -1; 
  public static final double kELevatorHeightMiddleNodeMeters = -1; //based on Arnav's calcs: 
  public static final double kELevatorHeightTopNodeMeters = -1; //based on Arnav's calcs: 
  public static final double kElevatorHeightBottomNodeMeters = -1; //based on Arnav's calcs: 
  public static double kElevatorTopHeightMeters = 0; 
  public static final double kElevatorBottomHeightMeters = 0; 
  public static final double kElevatorDistPerMotorEncoderTickMeters = (((2.0 * Math.PI * Units.inchesToMeters(0.6875))/12.8571429) / 2048); //TODO: Check if this calculation is correct
  public static final double kSpoolAbsEncoderDistancePerTickMeters = ((2.0*Math.PI* Units.inchesToMeters(0.6875))/2048);
  public static double kElevatorAbsEncoderZeroPositionMeters = 0;
  public static final double kMiddleNodeSetpoint = -1; 
  public static final double kTopNodeSetpoint = -1; 
  public static final double kElevatorMotorEncoderZeroingPower = -0.25; //TODO: This may need to be changed. 
  public static final double kElevatorMotorMaxHeightRecordingPower = 0.25; //TODO: This may need to be changed. 

  //GR: 12.8571429:1
  //1.375 = radius of spool(drum radius); 
}
