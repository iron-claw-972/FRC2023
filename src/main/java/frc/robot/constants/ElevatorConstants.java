package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public class ElevatorConstants {
  public static final int kMotorPort = 13;
  public static final int kTopLimitSwitchPort = 9;
  public static final int kBottomLimitSwitchPort = 8;

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

  public static final double kMiddleNodeHeightConeExtension = 0.65 ;
  public static final double kTopNodeHeightConeExtension = 1.36;
  public static final double kHybridNodeOuttakeExtension = 0.14;
  public static final double kGroundIntakeExtension = 0.14;
  public static final double kMiddleNodeHeightCubeExtension = 0.1;
  public static final double kTopNodeHeightCubeExtension = 0.1;
  public static final double kShelfIntakeHeightExtension = 0.1;

  public static final double kMinExtension = 0.0; 

  public static final double kGearRatio = (50.0/12.0)*(50.0/30.0)*(36.0/24.0);
  public static final double kCordThicknessInches = 0.09;
  public static final double kSpoolRadius = Units.inchesToMeters(0.6875+kCordThicknessInches); 
  public static final double kDistPerPulse = 2.0 * Math.PI * kSpoolRadius / kGearRatio / FalconConstants.kResolution;

  public static final double kCalibrationPower = 0.2; 
  /** Normal full power level for the elevator motor. */
  public static final double kPowerLimit = 0.1; //1

}

