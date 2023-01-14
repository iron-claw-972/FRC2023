package frc.robot.util;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.constants.Constants;

public class MotorFactory {

  private static int talonSRXDefaultContinuousLimit = 38;
  private static int kTalonSRXDefaultPeakLimit = 45;
  private static int kTalonSRXDefaultPeakDuration = 125;


  /*
  * Talon Sator / Supply Limits explanation 
  * Supply current is current that’s being drawn at the input bus voltage. Stator current is current that’s being drawn by the motor.
  * Supply limiting (supported by Talon SRX and FX) is useful for preventing breakers from tripping in the PDP.
  * Stator limiting (supported by Talon FX) is useful for limiting acceleration/heat.
  */

  public static final boolean kTalonFXStatorLimitEnable = false; // enabled?
  public static final double kTalonFXStatorCurrentLimit = 100; // Limit(amp)
  public static final double kTalonFXStatorTriggerThreshold = 100; // Trigger Threshold(amp)
  public static final double kTalonFXStatorTriggerDuration = 0; // Trigger Threshold Time(s)

  public static final boolean kTalonFXSupplyLimitEnable = false;  // enabled?
  public static final double kTalonFXSupplyCurrentLimit = 40;     // Limit(amp), usual current to hold after trigger hit
  public static final double kTalonFXSupplyTriggerThreshold = 55; // Trigger Threshold(amp), amps to activate trigger 
  public static final double kTalonFXSupplyTriggerDuration = 3; // Trigger Threshold Time(s), how long after trigger before reducing

  private static int kSparkMAXDefaultCurrentLimit = 60;

  private static double kVoltageCompensation = Constants.kMaxVoltage; 

  /**
   * Create a TalonSRX with current limiting enabled, using parameters. It will have current limiting and voltage compensation enabled and be set to Brake Mode.
   * 
   * @param id the ID of the TalonSRX
   * @param continuousCurrentLimit the continuous current limit to set in amps (A)
   * @param peakCurrentLimit the peak current limit to set in amps (A)
   * @param peakCurrentDuration the peak current limit duration to set in milliseconds (ms)
   * 
   * @return a fully configured TalonSRX object
   */
  public static WPI_TalonSRX createTalonSRX(int id, int continuousCurrentLimit, int peakCurrentLimit, int peakCurrentDuration) {
    TalonSRXConfiguration config = new TalonSRXConfiguration();
    config.continuousCurrentLimit = continuousCurrentLimit;
    config.peakCurrentLimit = peakCurrentLimit;
    config.peakCurrentDuration = peakCurrentDuration;
    config.voltageCompSaturation = kVoltageCompensation;

    WPI_TalonSRX talon = new WPI_TalonSRX(id);
    talon.configFactoryDefault();
    talon.configAllSettings(config);
    talon.enableCurrentLimit(true);
    talon.enableVoltageCompensation(true);
    talon.setNeutralMode(NeutralMode.Brake);

    return talon;
  }

  /**
   * Create a TalonSRX using default current limits.
   * 
   * @param id the ID of the TalonSRX
   * 
   * @return a fully configured TalonSRX object 
   */
  public static WPI_TalonSRX createTalonSRX(int id) {
    WPI_TalonSRX talon = createTalonSRX(id, talonSRXDefaultContinuousLimit, kTalonSRXDefaultPeakLimit, kTalonSRXDefaultPeakDuration);
    talon.enableCurrentLimit(true);

    return talon;
  }

  /**
   * Create a CANSparkMax with current limiting enabled
   * 
   * @param id the ID of the Spark MAX
   * @param motortype the type of motor the Spark MAX is connected to 
   * @param stallLimit the current limit to set at stall
   * 
   * @return a fully configured CANSparkMAX
   */
  public static CANSparkMax createSparkMAX(int id, MotorType motortype, int stallLimit) {
    CANSparkMax sparkMAX = new CANSparkMax(id, motortype);
    sparkMAX.restoreFactoryDefaults();
    sparkMAX.enableVoltageCompensation(kVoltageCompensation);
    sparkMAX.setSmartCurrentLimit(stallLimit);
    sparkMAX.setIdleMode(IdleMode.kBrake);

    sparkMAX.burnFlash();
    return sparkMAX;
  }

  /**
  * Create a CANSparkMax with default current limiting enabled
  * 
  * @param id the ID of the Spark MAX
  * @param motortype the type of motor the Spark MAX is connected to
  * 
  * @return a fully configured CANSparkMAX
  */
  public static CANSparkMax createSparkMAX(int id, MotorType motortype) {
    return createSparkMAX(id, motortype, kSparkMAXDefaultCurrentLimit);
  }

  /**
  * Create a configured TalonFX with all the default settings.
  * https://motors.vex.com/vexpro-motors/falcon
  * 
  * @param id the ID of the motor
  * @param canBus Name of the CANbus; can be a CANivore device name or serial number. Pass in "rio" to use the roboRIO.
  * 
  * @return a fully configured TalonFX
  */
  public static WPI_TalonFX createTalonFX(int id, String canBus) {

    if (id == -1) return null;

    TalonFXConfiguration config = new TalonFXConfiguration();
    
    config.statorCurrLimit = new StatorCurrentLimitConfiguration(
     // enabled                  | Limit(amp)               | Trigger Threshold(amp) |       Trigger Threshold Time(s)  */
        kTalonFXStatorLimitEnable, kTalonFXStatorCurrentLimit, kTalonFXStatorTriggerThreshold, kTalonFXStatorTriggerDuration);
    config.supplyCurrLimit = new SupplyCurrentLimitConfiguration(
        kTalonFXSupplyLimitEnable, kTalonFXSupplyCurrentLimit, kTalonFXSupplyTriggerThreshold, kTalonFXSupplyTriggerDuration);
    
    config.voltageCompSaturation = Constants.kMaxVoltage;

    WPI_TalonFX talon = new WPI_TalonFX(id, canBus);
    talon.configFactoryDefault();
    talon.configAllSettings(config);
    talon.enableVoltageCompensation(false);
    talon.setNeutralMode(NeutralMode.Coast);
    talon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    return talon;
  }

  /**
  * Create a configured TalonFX.
  * https://motors.vex.com/vexpro-motors/falcon
  * 
  * @param id the ID of the motor
  * @param canBus Name of the CANbus; can be a CANivore device name or serial number. Pass in "rio" to use the roboRIO.
  * @param supplyCurrentLimit the regular current to return to after the trigger
  * @param supplyTriggerThreshold The current at which the trigger will activate
  * @param supplyTriggerDuration The amount of time the current must be above the trigger current to reduce current
  * @param neutralMode Whether the motor is in coast or brake mode
  *
  * @return a fully configured TalonFX
  */
  public static WPI_TalonFX createTalonFX(int id, String canBus, double supplyCurrentLimit, double supplyTriggerThreshold, double supplyTriggerDuration , NeutralMode neutralMode) {

    if (id == -1) return null;

    TalonFXConfiguration config = new TalonFXConfiguration();
    
    config.supplyCurrLimit = new SupplyCurrentLimitConfiguration(
        true, supplyCurrentLimit, supplyTriggerThreshold, supplyTriggerDuration);
    
    config.voltageCompSaturation = Constants.kMaxVoltage;

    WPI_TalonFX talon = new WPI_TalonFX(id, canBus);
    talon.configFactoryDefault();
    talon.configAllSettings(config);
    talon.enableVoltageCompensation(false);
    talon.setNeutralMode(neutralMode);
    talon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    return talon;
  }

}