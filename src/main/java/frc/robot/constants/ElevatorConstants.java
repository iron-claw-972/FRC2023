package frc.robot.constants;

public class ElevatorConstants {
  public final int kElevatorMotor = -1; 
  public final double kP = -1; 
  public final double kI = -1; 
  public final double kD = -1; 
  public final double kELevatorHeightMiddleNodeInches = -1; //based on Arnav's calcs: 
  public final double kELevatorHeightTopNodeInches = -1; //based on Arnav's calcs: 
  public double kElevatorTopHeightInches = 0; 
  public final double kElevatorDistPerMotorEncoderTick = (((2.0 * Math.PI * 1.375)/12.8571429) / 2048); //TODO: Check if this calculation is correct
  public final double kMiddleNodeSetpoint = -1; 
  public final double kTopNodeSetpoint = -1; 
  //GR: 12.8571429:1
  //1.375 = radius of spool(drum radius); 

  


}
