package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.util.Conversions;
import frc.robot.util.DrawMechanism;
import frc.robot.util.LogManager;


public class Elevator extends SubsystemBase {
  private final ShuffleboardTab m_elevatorTab;

  private final BooleanSupplier m_hasConeSupplier;
  private ElevatorMode m_mode;
  private ElevatorStatus m_status;

  private final WPI_TalonFX m_motor;
  private final DigitalInput m_bottomLimitSwitch;
  private final DigitalInput m_topLimitSwitch;
  private double m_desiredPosition = 0.02;
  private double m_desiredPower = 0;
  private boolean m_isCalibrated;

  private double m_gravityCompensation = 0;

  public Elevator(ShuffleboardTab elevatorTab, BooleanSupplier hasConeSupplier) {
    m_elevatorTab = elevatorTab; 

    m_hasConeSupplier = hasConeSupplier;
    m_mode = ElevatorMode.DISABLED;
    m_status = ElevatorStatus.NONE;
    m_isCalibrated = false;

    m_motor = new WPI_TalonFX(ElevatorConstants.kMotorID, Constants.kCanivoreCAN);
    configElevatorMotor();

    m_bottomLimitSwitch = new DigitalInput(ElevatorConstants.kBottomLimitSwitchPort);
    m_topLimitSwitch = new DigitalInput(ElevatorConstants.kTopLimitSwitchPort);

    setupShuffleboard();
  }

  private void configElevatorMotor() {
    m_motor.configFactoryDefault();
    m_motor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(
      ElevatorConstants.kEnableCurrentLimit,
      ElevatorConstants.kContinuousCurrentLimit,
      ElevatorConstants.kPeakCurrentLimit,
      ElevatorConstants.kPeakCurrentDuration
    ));
    
    //configure PIDS and feedforwards within motor(not RoboRiO)
    //each PID is for a different condition
    m_motor.config_kP(0, ElevatorConstants.kBottomP);
    m_motor.config_kI(0, ElevatorConstants.kBottomI);
    m_motor.config_kD(0, ElevatorConstants.kBottomD);
    m_motor.config_kF(0, ElevatorConstants.kBottomF);

    m_motor.config_kP(1, ElevatorConstants.kBottomWithConeP);
    m_motor.config_kI(1, ElevatorConstants.kBottomWithConeI);
    m_motor.config_kD(1, ElevatorConstants.kBottomWithConeD);
    m_motor.config_kF(1, ElevatorConstants.kBottomWithConeF);

    m_motor.config_kP(2, ElevatorConstants.kTopP);
    m_motor.config_kI(2, ElevatorConstants.kTopI);
    m_motor.config_kD(2, ElevatorConstants.kTopD);
    m_motor.config_kF(2, ElevatorConstants.kTopF);

    m_motor.config_kP(3, ElevatorConstants.kTopWithConeP);
    m_motor.config_kI(3, ElevatorConstants.kTopWithConeI);
    m_motor.config_kD(3, ElevatorConstants.kTopWithConeD);
    m_motor.config_kF(3, ElevatorConstants.kTopWithConeF);

    m_motor.setInverted(ElevatorConstants.kMotorInvert);
    m_motor.setNeutralMode(ElevatorConstants.kNeutralMode);
    m_motor.configVoltageCompSaturation(Constants.kRobotVoltage);
    m_motor.enableVoltageCompensation(true);

    // m_motor.configClosedloopRamp(ElevatorConstants.kMotorRamp);

    m_motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    m_motor.configForwardSoftLimitThreshold(
      Conversions.MetersToFalcon(
        ElevatorConstants.kMaxPosition,
        ElevatorConstants.kSpoolCircumference,
        ElevatorConstants.kGearRatio
      )
    );
    m_motor.configReverseSoftLimitThreshold(Units.inchesToMeters(0));

    toggleSoftLimits(false);
  }

  /**
   * Turn on or off soft limits for the motors -- encoder positions
   * the motor will be set to neutral mode once it passes the limits
   * 
   * @param enabled
   * @return none
   */
  public void toggleSoftLimits(boolean enabled) {
    m_motor.configForwardSoftLimitEnable(enabled);
    m_motor.configReverseSoftLimitEnable(enabled);
  }

  /**
   * Checks if top limit switch is reached
   * @return true or false
   */
  public boolean isTopLimitSwitchReached() {
    return m_topLimitSwitch.get() != ElevatorConstants.kTopLimitSwitchNC;
  }

  /**
   * Checks if bottom limit switch is reached
   * @return true or false
   */
  public boolean isBottomLimitSwitchReached() {
    return m_bottomLimitSwitch.get() != ElevatorConstants.kBottomLimitSwitchNC;
  }

  /**
   * Cap the peak motor power that the talonFX will send to the falcon motor (forward and reverse). 
   * @param power
   * @return none
   */
  public void setMaxOutput(double power) {
    m_motor.configPeakOutputForward(power);
    m_motor.configPeakOutputReverse(power);
  }

  /**
   * Using the elevator position/extension calculate the actual height of the elevator
   * from the ground to the bottom of the carriage
   * 
   * @return elevator height from ground to bottom of carriage
   */
  public double getHeight() {
    return Conversions.ElevatorExtensionToHeight(getPosition());
  }

  /**
   * Set the desired elevator height from the ground. Do this by converting the elevator height to the
   * required position/extension, then passing in the value to the setDesiredPosition function (Defined above)
   * @param desiredHeight the height we want the elevator to move to from the ground
   */
  public void setDesiredHeight(double desiredHeight) {
    setDesiredPosition(Conversions.ElevatorHeightToExtension(desiredHeight));
  }

  /**
   * Get position of carriage above bottom position. AKA extension
   * @return position (m)
   */
  public double getPosition() {
    // in simulation, assume we are at the desired position
    if (RobotBase.isSimulation()) return m_desiredPosition;
    // calculate extension based on falcon encoder
    return Conversions.falconToMeters(
      m_motor.getSelectedSensorPosition(),
      ElevatorConstants.kSpoolCircumference,
      ElevatorConstants.kGearRatio
    );
  }

  /**
   * set the desired power to the m_desiredPower variable
   * @param power
   */
  public void setDesiredPosition(double desiredPosition) {
    m_desiredPosition = desiredPosition;
    if (m_mode == ElevatorMode.POSITION && m_isCalibrated && RobotBase.isSimulation())
      DrawMechanism.getInstance().setElevatorExtension(m_desiredPosition);
  }

  public void setDesiredPower(double power) {
    m_desiredPower = power;
  }

  /**
   * Reset/zero the inbuilt falcon motor encoder
   */
  public void zeroEncoder() {
    m_motor.setSelectedSensorPosition(0.0);
  }

  /**
   * If we are not calibrated and we are trying to set the elevator mode to 
   * something other than CALIBRATION Or DISABLED, don't do anything.
   * 
   * If we are calibrated, set the mode to whatever we desire. 
   * 
   * @param mode
   */
  public void setMode(ElevatorMode mode) {
    if (!m_isCalibrated && !(mode == ElevatorMode.CALIBRATION || mode == ElevatorMode.DISABLED)) return;
    m_mode = mode;
    if (m_mode == ElevatorMode.POSITION)
      DrawMechanism.getInstance().setElevatorExtension(m_desiredPosition);
  }

  /**
   * Get the velocity of the elevator in meters per seocnd
   * @return elevator velocity
   */
  public double getVelocity() {
    return Conversions.falconToMPS(
      m_motor.getSelectedSensorVelocity(),
      ElevatorConstants.kSpoolCircumference,
      ElevatorConstants.kGearRatio
    );
  }
  
  /**
   * Calculate whether or not the current elevator position is within a set range of the desired position and that
   * the velocity/speed of the elevator is less than a set velocity tolerance. 
   * @return true or false-- whether or not the elevator has reached it's desired position

   */
  public boolean reachedDesiredPosition() {
    return Math.abs(m_desiredPosition - getPosition()) < ElevatorConstants.kPositionTolerance
          && Math.abs(getVelocity()) < ElevatorConstants.kVelocityTolerance;
  }

  /*
   * Create enums for the different elevator positions:
   *    Bottom without cone
   *    Bottom with cone
   *    Top without cone
   *    Top with cone
   *    None
   */

  enum ElevatorStatus {
    BOTTOM, BOTTOM_CONE, TOP, TOP_CONE, NONE
  }
  
  /* 
   * Create enums for the elevator mode, whether we want the elevator to calibrate, 
   * use manual control, position control, or if we want the elevator to be disabled 
   */
  public enum ElevatorMode {
    CALIBRATION, MANUAL, POSITION, DISABLED
  }

  /**
   * If our elevator position is less than the distance that the carraige can travel in the first stage, 
   * check whether hasCone is true or false. If hasCone is true, set m_status to the ElevatorStatus enum BOTTOM_CONE. 
   * If hasCone is false, set m_status to the ElevatorStatus enum BOTTOM. 
   * 
   * <p>
   * 
   * If our elelevator position is more than the distance that the carriage can travel in the first stage, check whether hasCone is
   * true or false. If hasCone is true, set m_status to the ElevatorStatus enum TOP_CONE. If it's false, set m_status to the ElevatorStatus enum TOP.
  */
  private void updateElevatorStatus() {
    boolean hasCone = m_hasConeSupplier.getAsBoolean();
    if (getPosition() < ElevatorConstants.kCarriageMaxDistance) {
      m_status = hasCone ? ElevatorStatus.BOTTOM_CONE : ElevatorStatus.BOTTOM;
    } else {
      m_status = hasCone ? ElevatorStatus.TOP_CONE : ElevatorStatus.TOP;
    }
  }

  /**
   * Sepending on the elevator status select the right PID and 
   * set the right gravity compensation constant to m_gravityCompensation
   */
  private void updateClosedLoopSlot() {
    switch (m_status) {
      case BOTTOM:
        m_motor.selectProfileSlot(0, 0);
        m_gravityCompensation = ElevatorConstants.kBottomGravityCompensation;
        break;
      case BOTTOM_CONE:
        m_motor.selectProfileSlot(1, 0);
        m_gravityCompensation = ElevatorConstants.kBottomWithConeGravityCompensation;
        break;
      case TOP:
        m_motor.selectProfileSlot(2, 0);
        m_gravityCompensation = ElevatorConstants.kTopGravityCompensation;
        break;
      case TOP_CONE:
        m_motor.selectProfileSlot(3, 0);
        m_gravityCompensation = ElevatorConstants.kTopWithConeGravityCompensation;
        break;
      case NONE:
        break;
    };
  }

  /**
   * If the elevator is calibrated, set m_isCalibrated to true
   */
  public void setIsCalibrated() {
    m_isCalibrated = true;
  }

  /**The periodic method for the subsystem, it runs forever from the moment that the robot is enabled */
  @Override
  public void periodic() {
    //calculate the position error. We will use this to stop the motors when the top or bottom limit switches are hit. 
    //The motor know the error already for PID usage.
    double positionError = m_desiredPosition - getPosition();
    if ((isBottomLimitSwitchReached() && (positionError < 0 || m_desiredPower < 0))
        || (isTopLimitSwitchReached() && (positionError > 0 || m_desiredPower > 0))) {
      m_motor.stopMotor();
      return;
    }
    //depending on the elevatorMode, do certain things.
    switch (m_mode) {
      case CALIBRATION:
        m_motor.set(ControlMode.PercentOutput, ElevatorConstants.kCalibrationPower);
        break;
      case DISABLED:
        m_motor.stopMotor();
        break;
      case MANUAL:
        if (!m_isCalibrated) break; //if the elevator is not calibrated don't do anything. 
        updateElevatorStatus();
        m_motor.set(ControlMode.PercentOutput, m_desiredPower);
        break;
      case POSITION:
        if (!m_isCalibrated) break; //if the elevator is not calibrated don't do anything. 
        updateElevatorStatus(); //set the m_status variable to the desired status requested by a command or trigger or something(not completely sure)
        updateClosedLoopSlot(); //select the right PIDs and set m_gravityCompenstion to the right gravity compensation variable
        m_motor.set( 
          ControlMode.Position, //type of control we want to use(PID control is position control, so use ControlMode.Position)
          Conversions.MetersToFalcon(m_desiredPosition, ElevatorConstants.kSpoolCircumference, ElevatorConstants.kGearRatio), //process variable(elevator position/extension)
          DemandType.ArbitraryFeedForward, //type of feedforward
          m_gravityCompensation // put in the m_gravityCompensation variable
        );
        break;
    }
    
    if (Constants.kLogging) updateLogs(); //if we are logging(Constants.kLogging is set to true), update the elevator logs
  }

  private void setupShuffleboard() {
    if (Constants.kUseTelemetry) { 
      //if we are using telemetry(Constants.kUseTelemetry is set to true), put a bunch of stuff on the elevatorTab on shuffleboard
      m_elevatorTab.addDouble("Current Position (m)", this::getPosition);
      m_elevatorTab.addDouble("Current Height (m)", this::getHeight);
      m_elevatorTab.addDouble("Desired Position (m)", () -> m_desiredPosition);
      m_elevatorTab.addDouble("Desired Height (m)", () -> Conversions.ElevatorExtensionToHeight(m_desiredPosition));
      m_elevatorTab.addDouble("Desired Power", () -> m_desiredPower);
      m_elevatorTab.addBoolean("Is Calibrated", () -> m_isCalibrated);
      m_elevatorTab.addBoolean("Reached desired position", this::reachedDesiredPosition);
      m_elevatorTab.addBoolean("Reached Top Limit Switch", this::isTopLimitSwitchReached);
      m_elevatorTab.addBoolean("Reached Bottom Limit Switch", this::isBottomLimitSwitchReached);
      m_elevatorTab.addDouble("Supply Current (A)", m_motor::getSupplyCurrent);
      m_elevatorTab.addDouble("Stator Current (A)", m_motor::getStatorCurrent);
      m_elevatorTab.addDouble("Commanded power", m_motor::get);
      m_elevatorTab.addString("Mode", () -> m_mode.toString());
      m_elevatorTab.addString("Status", () -> m_status.toString());
      m_elevatorTab.addDouble("Gravity compensation", () -> m_gravityCompensation);
    }
  }

  public void updateLogs() {
    //update the elevator logs
    LogManager.addDouble("Elevator/desiredPosition", m_desiredPosition);
    LogManager.addDouble("Elevator/extension", getPosition());
    LogManager.addBoolean("Elevator/bottomLimitSwitch", isBottomLimitSwitchReached());
    LogManager.addBoolean("Elevator/topLimitSwitch", isTopLimitSwitchReached());
  }
}
