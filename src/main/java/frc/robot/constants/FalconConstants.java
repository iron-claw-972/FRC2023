package frc.robot.constants;

public class FalconConstants {

  // Stored in hex though not really a hex. 21.0 = 0x2100, ex. 1.2 = 0x0102
  public final int kFirmwareVersion = 0x2100;
  public final boolean kBreakOnWrongFirmware = true;

  public final double kResolution = 2048;
  public final double kMaxRpm = 6380.0; // Rotations per minute

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
  public final boolean kStatorLimitEnable = false; // enabled?
  public final double kStatorCurrentLimit = 100; // Limit(amp)
  public final double kStatorTriggerThreshold = 100; // Trigger Threshold(amp)
  public final double kStatorTriggerDuration = 0; // Trigger Threshold Time(s)

  // Supply
  public final boolean kSupplyLimitEnable = false; // enabled?
  public final double kSupplyCurrentLimit = 40; // Limit(amp), current to hold after trigger hit
  public final double kSupplyTriggerThreshold = 55; // (amp), amps to activate trigger
  public final double kSupplyTriggerDuration = 3; // (s), how long after trigger before reducing

}
