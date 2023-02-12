package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public class ElevatorConstants {
  //TODO: Set everything to meters in whole robot code
  public final int kElevatorMotor = -1; 
  public final double kElevatorP = -1; 
  public final double kElevatorI = -1; 
  public final double kElevatorD = -1; 
  public final double kELevatorHeightMiddleNodeMeters = -1; //based on Arnav's calcs: 
  public final double kELevatorHeightTopNodeMeters = -1; //based on Arnav's calcs: 
  public final double kElevatorHeightBottomNodeMeters = -1; //based on Arnav's calcs: 
  public double kElevatorTopHeightMeters = 0; 
  public final double kElevatorBottomHeightMeters = 0; 
  public final double kElevatorDistPerMotorEncoderTickMeters = (((2.0 * Math.PI * Units.inchesToMeters(0.6875))/12.8571429) / 2048); //TODO: Check if this calculation is correct
  public final double kSpoolAbsEncoderDistancePerTickMeters = ((2.0*Math.PI* Units.inchesToMeters(0.6875))/2048);
  public double kElevatorAbsEncoderZeroPositionMeters = 0;
  public final double kMiddleNodeSetpoint = -1; 
  public final double kTopNodeSetpoint = -1; 
  //GR: 12.8571429:1
  //1.375 = radius of spool(drum radius); 

  


}
