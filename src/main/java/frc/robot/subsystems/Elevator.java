/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.util.LogManager;
import frc.robot.util.MotorFactory;
import lib.ctre_shims.TalonEncoder;

public class Elevator extends SubsystemBase {

  private final WPI_TalonFX m_motor;
  private final PIDController m_elevatorPID;
  private final TalonEncoder m_talonEncoder; 
  private final DigitalInput m_topLimitSwitch; 
  private final DigitalInput m_bottomLimitSwitch; 
  private final ShuffleboardTab m_elevatorTab; 

  private boolean m_enabled; 
  private boolean m_isCalibrated = false; 

  public Elevator(ShuffleboardTab elevatorTab) {
    m_elevatorTab = elevatorTab; 
    m_motor = MotorFactory.createTalonFX(ElevatorConstants.kMotorPort, Constants.kCanivoreCAN);
    m_motor.setInverted(true);

    m_talonEncoder = new TalonEncoder(m_motor); 
    m_talonEncoder.setDistancePerPulse(ElevatorConstants.kDistPerPulse);
    addChild("motor", m_motor);
   
    m_elevatorPID = new PIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);
    m_elevatorPID.setTolerance(ElevatorConstants.kTolerance);

    m_topLimitSwitch = new DigitalInput(ElevatorConstants.kTopLimitSwitchPort); 
    m_bottomLimitSwitch = new DigitalInput(ElevatorConstants.kBottomLimitSwitchPort); 
    addChild("Upper Limit",m_topLimitSwitch);
    addChild("Bottom Limit",m_bottomLimitSwitch);
    
    setUpElevatorTab();
  }

  @Override
  public void periodic() {
    if(m_enabled && m_isCalibrated) {
      double pidPower = m_elevatorPID.calculate(getExtension(), MathUtil.clamp(m_elevatorPID.getSetpoint(), ElevatorConstants.kMinExtension, ElevatorConstants.kMaxExtension));
      setMotorPower(pidPower);
    }
  }

  public void close() {
    // close the ports
    m_topLimitSwitch.close(); 
    m_bottomLimitSwitch.close(); 
  }

  /**
   * Set the motor power while taking the limit switches into account. 
   * If the bottom limit switch is tripped, then the elevator is only allowed to move up. 
   * If the top limit switch is tripped, then the elevator is inly allowed to move down.
   * The motor power will also be clamped to the max power constant value.
   * @param power the power we want to give to the motor
   */
  public void setMotorPower(double power) {
    if ((isBottomSwitchTripped() && power < 0) || (isTopSwitchTripped() && power > 0)) {
      m_motor.set(0); 
    } else {
      m_motor.set(MathUtil.clamp(power, -ElevatorConstants.kPowerLimit, ElevatorConstants.kPowerLimit));
    }
  }
  
  public void setPIDEnabled(boolean isEnabled) {
    m_enabled = isEnabled; 
  }

  public void setIsCalibrated() {
    m_isCalibrated = true; 
  }

  public void setTargetExtension(double setpoint) {
    if(setpoint > ElevatorConstants.kMaxExtension) {
      System.out.println("SETPOINT IS MORE THAN THE MAX EXTENSION!!!!!!!!!!!"); 
      return; 
    }

    if(setpoint <0) {
      System.out.println("SETPOINT IS LESS THAN THE MIN EXTENSION!!!!!!!!!!!"); 
      return; 
    }

    m_elevatorPID.setSetpoint(setpoint);
    m_elevatorPID.reset(); 
  }

  public boolean isBottomSwitchTripped() {
    return !m_bottomLimitSwitch.get();
  }

  public boolean isTopSwitchTripped() {
    return !m_topLimitSwitch.get();
  }

  public void resetTalonEncoder() {
    m_talonEncoder.reset(); 
  }
 
  /**
   * Determine the elevator extension from the motor encoder.
   * @return return extension in meters
   */
  public double getExtension() {
    return m_talonEncoder.getDistance(); 
  }

  public boolean atSetpoint() {
    return m_elevatorPID.atSetpoint();
  }

  /**
   * @return the error, in meters, from the current PID setpoint. If the PID is disabled, then the error is zero. Positive error indicates it is above the desired position.
   */
  public double getError() {
    if (!m_enabled) return 0;
    return getExtension() - m_elevatorPID.getSetpoint();
  }

  public void setUpElevatorTab() {
    m_elevatorTab.addNumber("Elevator Extension", () -> getExtension());
    m_elevatorTab.add(m_elevatorPID); 
    m_elevatorTab.addBoolean("enabled", () -> m_enabled);
    m_elevatorTab.addBoolean("topLimitSwitch", () -> isTopSwitchTripped() );
    m_elevatorTab.addBoolean("bottomLimitSwitch", () -> isBottomSwitchTripped());
  }

  public void logging(){
    LogManager.addDouble("Elevator/error", () -> getError());
    LogManager.addDouble("Elevator/extension", () -> getExtension());
    LogManager.addBoolean("Elevator/bottomLimitSwitch", () -> isBottomSwitchTripped());
    LogManager.addBoolean("Elevator/topLimitSwitch", () -> isTopSwitchTripped());
  }
}

