package frc.robot.constants;

import frc.robot.util.RobotType;

public final class Constants {
  public static final double kGravitationalAccel = 9.8;
  public static final double kMaxVoltage = 12.0;
  public static final double kLoopTime = 0.02;
  
  public static final double kCancoderResolution = 4096;

  public static final RobotType kRobotType = RobotType.COMP;

  // CAN bus names
  // Replaced for DriveConstants.xxxxCAN constants
  @Deprecated
  public static final String kCanivoreCAN = "CANivore";
  @Deprecated
  public static final String kRioCAN = "rio";

  @Deprecated
  public static final OIConstants oi = new OIConstants();
  @Deprecated
  public static final AutoConstants auto = new AutoConstants();
  @Deprecated
  public static final FalconConstants falcon = new FalconConstants();
}
