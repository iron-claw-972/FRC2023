package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public class ElevatorConstants {
  //TODO: Set everything to meters in whole robot code
  public static final int kMotorPort = 13;
  public static final int kTopLimitSwitchPort = 9;
  public static final int kBottomLimitSwitchPort = 8;
  public static final int kAbsEncoderPort = 6;

  public static final double kP = 5;
  public static final double kI = 0.005;
  public static final double kD = 0.0;

  //these feedforward values were calculated using : https://www.reca.lc/linear
  public static final double kS = 0.0000001; //FIXME: Not sure what to put for this feedforward value. 
  public static final double kG = 0.28; 
  public static final double kV = 10.72; 
  public static final double kA = 0.05; 

  public static final double kVelocity = 0.25; 
  public static final double kAccel = 0; 

  // Game constants
  /** Middle Node height (in meters) */
  public static final double kMiddleNodeHeightCone = Units.inchesToMeters(24.5);
  public static final double kTopNodeHeightCone = Units.inchesToMeters(46);
  public static final double kMiddleNodeHeightCube = Units.inchesToMeters(23.5);
  public static final double kTopNodeHeightCube = Units.inchesToMeters(46);
  public static final double kShelfIntakeHeight = Units.inchesToMeters(37.375);

  /** elevator travel distance in meters**/

  public static final double kMiddleNodeHeightConeTravel = calculateElevatorPosition(kMiddleNodeHeightCone);
  public static final double kTopNodeHeightConeTravel = calculateElevatorPosition(kTopNodeHeightCone);
  public static final double kMiddleNodeHeightCubeTravel = calculateElevatorPosition(kMiddleNodeHeightCube);
  public static final double kTopNodeHeightCubeTravel = calculateElevatorPosition(kTopNodeHeightCube);
  public static final double kShelfIntakeHeightTravel = calculateElevatorPosition(kShelfIntakeHeight);

  public static final double kMinHeight = 0.0; 

  public static final double kGearRatio = (50.0/12.0)*(50.0/30.0)*(36.0/24.0); //TODO: put in actual GR
  public static final double kCordThicknessInches = 0.09;
  public static final double kSpoolRadius = Units.inchesToMeters(0.6875+kCordThicknessInches); 
  public static final double kDistPerPulse = 2.0 * Math.PI * kSpoolRadius / kGearRatio / FalconConstants.kResolution;

  public static final double kCalibrationPower = 0.2; 
  /** Normal full power level for the elevator motor. */
  public static final double kPowerLimit = 0.1; //1

  /**
   * Calculate the elevator extension.
   * @param height height above the floor in meters.
   * @returns elevator extension in meters.
   */
  private static double  calculateElevatorPosition(double height) {
    return (height+Units.metersToInches(5.5))/(Math.sin(Math.toRadians(55))); 
  }

}
