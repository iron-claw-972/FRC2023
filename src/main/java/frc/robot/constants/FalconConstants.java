package frc.robot.constants;

public class FalconConstants {

  public static final int kFirmwareVersion = 5633; // version 22.1.1.0
  public static final boolean kBreakOnWrongFirmware = false; // TODO: fix issue that make the robot break

  public static final double kResolution = 2048;
  public static final double kMaxRpm = 6380.0; // Rotations per minute

  
  /*
   * Talon Stator / Supply Limits explanation
   * Supply current is current that’s being drawn at the input bus voltage. Stator
   * current is current that’s being drawn by the motor.
   * Supply limiting (supported by Talon FX and SRX) is useful for preventing
   * breakers from tripping in the PDP.
   * Stator limiting (supported by Talon FX) is useful for limiting
   * acceleration/heat.
   */

  // These are the default values

  // Stator
  public static final boolean kStatorLimitEnable = false; // enabled?
  public static final double kStatorCurrentLimit = 100; // Limit(amp)
  public static final double kStatorTriggerThreshold = 100; // Trigger Threshold(amp)
  public static final double kStatorTriggerDuration = 0; // Trigger Threshold Time(s)

  // Supply
  public static final boolean kSupplyLimitEnable = false; // enabled?
  public static final double kSupplyCurrentLimit = 40; // Limit(amp), current to hold after trigger hit
  public static final double kSupplyTriggerThreshold = 55; // (amp), amps to activate trigger
  public static final double kSupplyTriggerDuration = 3; // (s), how long after trigger before reducing

}