package frc.robot.constants;

import frc.robot.constants.swerve.SwerveDriveConstants;

public final class Constants {
  public static final double kGravitationalAccel = 9.8;
  public static final double kMaxVoltage = 12.0;
  public static final double kLoopTime = 0.02;
  
  public static final double kCancoderResolution = 4096;

  // CAN bus names
  public static final String kCanivoreCAN = "CANivore";
  public static final String kRioCAN = "rio";

  public static final FalconConstants falcon = new FalconConstants();
  public static final OIConstants oi = new OIConstants();
  public static final SwerveDriveConstants drive = SwerveDriveConstants.COMP;
  public static final AutoConstants auto = new AutoConstants();
}
