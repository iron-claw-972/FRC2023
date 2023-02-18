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

  public static final double kHeightBottomNode = 0.2; //based on Arnav's calcs: 
  public static final double kHeightMiddleNode = 0.3; //based on Arnav's calcs: 
  public static final double kHeightTopNode = 0.4; //based on Arnav's calcs: 
  public static final double kMaxHeight = 0.5; 
  public static final double kMinHeight = 0.0; 

  public static final double kGearRatio = (double) 90 / 7; //TODO: put in actual GR
  public static final double kSpoolRadius = Units.inchesToMeters(0.6875); 
  public static final double kDistPerPulse = 2.0 * Math.PI * kSpoolRadius / kGearRatio / FalconConstants.kResolution;

  public static final double kCalibrationPower = 0.25;
  /** Normal full power level for the elevator motor. */
  public static final double kPowerLimit = 0.30;

}
