package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public class ElevatorConstants {
  public static final int kMotorPort = 13;
  public static final int kTopLimitSwitchPort = 9;
  public static final int kBottomLimitSwitchPort = 8;

  public static final double kP = 1.2;
  public static final double kI = 0.3;
  public static final double kD = 0.0;

  // Game constants
  /** Middle Node height (in meters) */
  public static final double kMiddleNodeHeightCone = Units.inchesToMeters(24.5);
  public static final double kTopNodeHeightCone = Units.inchesToMeters(34.5);
  
  public static final double kMiddleNodeHeightCube = Units.inchesToMeters(23.5);
  public static final double kTopNodeHeightCube = Units.inchesToMeters(46);
  
  public static final double kShelfIntakeHeight = Units.inchesToMeters(37.375);

  /** elevator travel distance in meters**/
  public static final double kMiddleNodeHeightConeExtension = heightToElevatorExtension(kMiddleNodeHeightCone);//0.65;
  public static final double kTopNodeHeightConeExtension = heightToElevatorExtension(kTopNodeHeightCone); // 1.2; 
  
  public static final double kMiddleNodeHeightCubeExtension = heightToElevatorExtension(kMiddleNodeHeightCube);
  public static final double kTopNodeHeightCubeExtension = heightToElevatorExtension(kTopNodeHeightCube);  
  
  public static final double kHybridNodeOuttakeExtension = 0;
  public static final double kGroundIntakeExtension = 0;

  public static final double kShelfIntakeHeightExtension = heightToElevatorExtension(kShelfIntakeHeight); 

  public static final double kMaxExtension = 1.3; 
  public static final double kMinExtension = 0.0; 

  public static final double kGearRatio = (50.0/12.0)*(50.0/30.0)*(36.0/24.0);
  public static final double kCordThicknessInches = 0.125;
  public static final double kSpoolRadius = Units.inchesToMeters(0.6875+kCordThicknessInches); 
  public static final double kDistPerPulse = 2.0 * Math.PI * kSpoolRadius / kGearRatio / FalconConstants.kResolution;

  public static final double kCalibrationPower = 0.2; 
  /** Normal full power level for the elevator motor. */
  public static final double kPowerLimit = 0.2; //1

  public static double heightToElevatorExtension(double height){
    double extension = Units.inchesToMeters(height-10.5)/Math.sin(Units.degreesToRadians(55));
    return extension; 
  }
}

