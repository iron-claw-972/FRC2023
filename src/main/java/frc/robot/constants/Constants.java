package frc.robot.constants;

public final class Constants {
  public static final double kGravitationalAccel = 9.8;
  public static final double kMaxVoltage = 12.0;
  public static final double kLoopTime = 0.02;
  
  public static final double kCancoderResolution = 4096;

  // CAN bus names
  public static final String kCanivoreCAN = "CANivore";
  public static final String kRioCAN = "rio";

  public static final OIConstants oi = new OIConstants();
  public static final DriveConstants drive = new DriveConstants();
  public static final AutoConstants auto = new AutoConstants();
  public static final FalconConstants falcon = new FalconConstants();
}
