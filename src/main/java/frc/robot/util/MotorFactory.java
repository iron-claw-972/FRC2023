package frc.robot.util;

import java.io.IOException;

import java.io.IOError;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.Constants;

public class MotorFactory {

  private static int talonSRXDefaultContinuousLimit = 38;
  private static int kTalonSRXDefaultPeakLimit = 45;
  private static int kTalonSRXDefaultPeakDuration = 125;

  private static int kSparkMaxDefaultCurrentLimit = 60;

  ///////////////////////////////////////////////////////////////////////////////////////////////
  // TALON SRX
  ///////////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Create a TalonSRX with current limiting enabled, using parameters. It will
   * have current
   * limiting and voltage compensation enabled and be set to Brake Mode.
   *
   * @param id                     the ID of the TalonSRX
   * @param continuousCurrentLimit the continuous current limit to set in amps (A)
   * @param peakCurrentLimit       the peak current limit to set in amps (A)
   * @param peakCurrentDuration    the peak current limit duration to set in
   *                               milliseconds (ms)
   * @return a fully configured TalonSRX object
   */
  public static WPI_TalonSRX createTalonSRX(
      int id, int continuousCurrentLimit, int peakCurrentLimit, int peakCurrentDuration) {

    TalonSRXConfiguration config = new TalonSRXConfiguration();
    config.continuousCurrentLimit = continuousCurrentLimit;
    config.peakCurrentLimit = peakCurrentLimit;
    config.peakCurrentDuration = peakCurrentDuration;
    config.voltageCompSaturation = Constants.kMaxVoltage;

    WPI_TalonSRX talon = new WPI_TalonSRX(id);
    talon.configFactoryDefault();
    talon.configAllSettings(config);
    talon.enableCurrentLimit(true);
    talon.enableVoltageCompensation(false);
    talon.setNeutralMode(NeutralMode.Brake);

    LogManager.addDouble("TalonFX/Current/" + talon.getDeviceID(), () -> talon.getStatorCurrent());
    LogManager.addDouble("TalonFX/Temperature/" + talon.getDeviceID(), () -> talon.getTemperature());
    LogManager.addDouble("TalonFX/RPM/" + talon.getDeviceID(), () -> talon.getSelectedSensorVelocity() / Constants.falcon.kResolution * 10 * 60);

    return talon;
  }

  /**
   * Create a TalonSRX using default current limits.
   *
   * @param id the ID of the TalonSRX
   * @return a fully configured TalonSRX object
   */
  public static WPI_TalonSRX createTalonSRXDefault(int id) {
    return createTalonSRX(id, talonSRXDefaultContinuousLimit, kTalonSRXDefaultPeakLimit, kTalonSRXDefaultPeakDuration);
  }

  ///////////////////////////////////////////////////////////////////////////////////////////////
  // SPARK MAX
  ///////////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Create a CANSparkMax with current limiting enabled
   *
   * @param id         the ID of the Spark MAX
   * @param motortype  the type of motor the Spark MAX is connected to
   * @param stallLimit the current limit to set at stall
   * @return a fully configured CANSparkMAX
   */
  public static CANSparkMax createSparkMAX(int id, MotorType motortype, int stallLimit) {
    CANSparkMax sparkMAX = new CANSparkMax(id, motortype);
    sparkMAX.restoreFactoryDefaults();
    sparkMAX.enableVoltageCompensation(Constants.kMaxVoltage);
    sparkMAX.setSmartCurrentLimit(stallLimit);
    sparkMAX.setIdleMode(IdleMode.kBrake);

    sparkMAX.burnFlash();
    return sparkMAX;
  }

  /**
   * Create a CANSparkMax with default current limiting enabled
   *
   * @param id        the ID of the Spark MAX
   * @param motortype the type of motor the Spark MAX is connected to
   * @return a fully configured CANSparkMAX
   */
  public static CANSparkMax createSparkMAXDefault(int id, MotorType motortype) {
    return createSparkMAX(id, motortype, kSparkMaxDefaultCurrentLimit);
  }

  ///////////////////////////////////////////////////////////////////////////////////////////////
  // TALON FX (Falcon 500)
  ///////////////////////////////////////////////////////////////////////////////////////////////

  /**
   * 
   * Creates a TalonFX with all current limit options. If you would like to use
   * defaults it is recommended to use the other createTalonFX.. methods.
   * 
   * @param id                     the id of the the motor
   * @param CANBus                 the CAN Bus the motor is on
   * @param StatorLimitEnable      whether or not to enable stator limiting
   * @param StatorCurrentLimit     the current, in amps, to return to after the
   *                               stator limit is triggered
   * @param StatorTriggerThreshold the threshold current to trigger the stator
   *                               limit
   * @param StatorTriggerDuration  the duration, in seconds, the current is above
   *                               the threshold before triggering
   * @param SupplyLimitEnable      whether or not to enable supply limiting
   * @param SupplyCurrentLimit     the current, in amps, to return to after the
   *                               supply limit is triggered
   * @param SupplyTriggerThreshold the threshold current to trigger the supply
   *                               limit
   * @param SupplyTriggerDuration  the duration, in seconds, the current is above
   *                               the threshold before triggering
   * @return A fully configured TalonFX
   */
  public static WPI_TalonFX createTalonFXFull(int id, String CANBus, boolean StatorLimitEnable,
      double StatorCurrentLimit,
      double StatorTriggerThreshold, double StatorTriggerDuration, boolean SupplyLimitEnable, double SupplyCurrentLimit,
      double SupplyTriggerThreshold, double SupplyTriggerDuration) {

    if (id == -1) {
      return null;
    }

    WPI_TalonFX talon = new WPI_TalonFX(id, CANBus);

    if (talon.getFirmwareVersion() != Constants.falcon.kFirmwareVersion) {
      String errorMessage = "TalonFX " + id + " firmware incorrect. Has " + talon.getFirmwareVersion()
          + ", currently FalconConstants.java requires: " + Constants.falcon.kFirmwareVersion;
      if (Constants.falcon.kBreakOnWrongFirmware) {
        DriverStation.reportError(errorMessage, true);
        throw new IOError(new IOException(errorMessage));
      } else {
        DriverStation.reportWarning(errorMessage + ", ignoring due to user specification.", false);
      }
    }

    TalonFXConfiguration config = new TalonFXConfiguration();

    // See explanations for Supply and Stator limiting in FalconConstants.java
    config.statorCurrLimit = new StatorCurrentLimitConfiguration(StatorLimitEnable, StatorCurrentLimit,
        StatorTriggerThreshold, StatorTriggerDuration);
    config.supplyCurrLimit = new SupplyCurrentLimitConfiguration(SupplyLimitEnable, SupplyCurrentLimit,
        SupplyTriggerThreshold, SupplyTriggerDuration);

    config.voltageCompSaturation = Constants.kMaxVoltage;

    talon.configFactoryDefault();
    talon.configAllSettings(config);
    talon.enableVoltageCompensation(false);
    talon.setNeutralMode(NeutralMode.Brake);
    talon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    return talon;
  }

  /**
   * @param id
   * @param CANBus
   */
  public static WPI_TalonFX createTalonFX(int id, String CANBus) {
    return createTalonFXFull(id, CANBus, Constants.falcon.kStatorLimitEnable, Constants.falcon.kStatorCurrentLimit,
        Constants.falcon.kStatorTriggerThreshold, Constants.falcon.kStatorTriggerDuration,
        Constants.falcon.kSupplyLimitEnable, Constants.falcon.kSupplyCurrentLimit,
        Constants.falcon.kSupplyTriggerThreshold, Constants.falcon.kSupplyTriggerDuration);
  }

  public static WPI_TalonFX createTalonFXSupplyLimit(int id, String CANBus, double currentLimit,
      double triggerThreshold, double triggerDuration) {
    return createTalonFXFull(id, CANBus, Constants.falcon.kStatorLimitEnable, Constants.falcon.kStatorCurrentLimit,
        Constants.falcon.kStatorTriggerThreshold, Constants.falcon.kStatorTriggerDuration, true, currentLimit,
        triggerThreshold, triggerDuration);
  }

  public static WPI_TalonFX createTalonFXStatorLimit(int id, String CANBus, double currentLimit,
      double triggerThreshold, double triggerDuration) {
    return createTalonFXFull(id, CANBus, true, currentLimit, triggerThreshold, triggerDuration,
        Constants.falcon.kSupplyLimitEnable, Constants.falcon.kSupplyCurrentLimit,
        Constants.falcon.kSupplyTriggerThreshold, Constants.falcon.kSupplyTriggerDuration);
  }
}