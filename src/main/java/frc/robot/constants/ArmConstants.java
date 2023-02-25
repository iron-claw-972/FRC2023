package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public class ArmConstants {
  // TODO: arm ids - TBD (fake using 5 -- 1-4 have been taken)
  public static final int kMotorId = 5;
  public static final int kAbsEncoderId = 7; 

  public static final int kEncoderCountsPerRev = 8192;

  // TODO: PID values - TBD (fake values for now)
  public static final double kP = 1.0;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  public static final double kTolerance = Units.degreesToRotations(3);
  public static final double kMinMotorPower = -0.1;
  public static final double kMaxMotorPower = 0.1;

  // feedforward values - TBD
  public static final double kG = 0;
  public static final double kV = 0;
  
  // TODO: distance values - TBD (fake values for now)
  //public static final double kInitialPosition = 0.0;
  public static final double kShelfPosition = 0.6;
  public static final double kIntakePosition = 0.6;
  public static final double kMiddlePosition = 0.6;
  public static final double kTopPosition = 0.6;

  public static final double kGR = (45.0/1.0)*(18.0/16.0)*(48.0/32.0); 
  public static final double kBottomPosition = 0.809821; 
  public static final double kStowPosition = 0.2663766; 

}
