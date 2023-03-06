package frc.robot.constants;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;

import edu.wpi.first.math.util.Units;
import frc.robot.util.Conversions;

public class ElevatorConstants {
  public static final int kMotorID = 13;
    //public static final String kElevatorCAN = kCANivoreCAN;
    public static final TalonFXInvertType kMotorInvert = TalonFXInvertType.Clockwise; // CW goes up
    public static final NeutralMode kNeutralMode = NeutralMode.Brake;

    public static final double kGearRatio = (50.0 / 12.0) * (50.0 / 30.0) * (36.0 / 24.0); // 10.416:1
    public static final double kSpoolDiameter = Units.inchesToMeters(1.375);
    public static final double kStringThickness = Units.inchesToMeters(0.125);
    public static final double kSpoolCircumference = (kSpoolDiameter + kStringThickness) * Math.PI;

    public static final int kBottomLimitSwitchPort = 8;
    public static final int kTopLimitSwitchPort = 9;

    public static final double kPositionTolerance = 0.2;
    public static final double kVelocityTolerance = 0.05; // FIXME: Elevator
    
    // Whether limit switch is normally-closed (activated = open circuit) or normally-open (activated = closed circuit)
    public static final boolean kTopLimitSwitchNC = true;
    public static final boolean kBottomLimitSwitchNC = true;
    
    // Slot 0
    public static final double kBottomP = 0.05; // FIXME: Elevator
    public static final double kBottomI = 0;
    public static final double kBottomD = 0;
    public static final double kBottomF = 0;
    public static final double kBottomGravityCompensation = 0;

    // Slot 1
    public static final double kBottomWithConeP = kBottomP;
    public static final double kBottomWithConeI = kBottomI;
    public static final double kBottomWithConeD = kBottomD;
    public static final double kBottomWithConeF = kBottomF;
    public static final double kBottomWithConeGravityCompensation = kBottomGravityCompensation;

    // Slot 2
    public static final double kTopP = kBottomP;
    public static final double kTopI = kBottomI;
    public static final double kTopD = kBottomD;
    public static final double kTopF = kBottomF;
    public static final double kTopGravityCompensation = 0.2;

    // Slot 3
    public static final double kTopWithConeP = kTopP;
    public static final double kTopWithConeI = kTopI;
    public static final double kTopWithConeD = kTopD;
    public static final double kTopWithConeF = kTopF;
    public static final double kTopWithConeGravityCompensation = kTopGravityCompensation;

    public static final int kContinuousCurrentLimit = 30; // FIXME: Elevator
    public static final int kPeakCurrentLimit = 50;
    public static final double kPeakCurrentDuration = 0.1;
    public static final boolean kEnableCurrentLimit = false;

    // Max distance that the carriage can travel within the first stage
    public static final double kCarriageMaxDistance = Units.inchesToMeters(25 - 0.25); // The 0.25 inches is the bottom hardstop
    // Max distance that the first stage can travel within the base stage
    public static final double kFirstStageMaxDistance = Units.inchesToMeters(26);
    // Total max travel distance of elevator (how far it can extend)
    public static final double kMaxPosition = kCarriageMaxDistance + kFirstStageMaxDistance;
    // Max height of elevator
    public static final double kMaxHeight = Conversions.ElevatorExtensionToHeight(kMaxPosition);
    // Vertical height of the center of the top surface of the tread hardstop for the carriage when elevator is at minimum height
    public static final double kElevatorBaseHeight = Units.inchesToMeters(12.974338);
    // Angle of elevator from the horizontal axis
    public static final double kElevatorAngle = Units.degreesToRadians(55.0);

    public static final double kIntakeConeHeight = Conversions.ElevatorExtensionToHeight(Units.inchesToMeters(4));
    public static final double kIntakeCubeHeight = kIntakeConeHeight;
    public static final double kTopConeHeight = Conversions.ElevatorExtensionToHeight(Units.inchesToMeters(46));
    
    public static final double kTopCubeHeight = kTopConeHeight;
    public static final double kMiddleConeHeight = Units.inchesToMeters(35);
    public static final double kMiddleCubeHeight = kMiddleConeHeight;
    public static final double kBottomConeHeight = Conversions.ElevatorExtensionToHeight(Units.inchesToMeters(6));
    public static final double kBottomCubeHeight = kBottomConeHeight;
    public static final double kShelfConeHeight = Units.inchesToMeters(58.225);
    public static final double kShelfCubeHeight = kShelfConeHeight;
    public static final double kStowHeight = Conversions.ElevatorExtensionToHeight(Units.inchesToMeters(0));

    public static final double kCalibrationPower = -0.2;
    public static final double kMotorRamp = 0.2;
}

