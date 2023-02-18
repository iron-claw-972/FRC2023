/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;
import java.util.logging.LogManager;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.util.MotorFactory;
import lib.ctre_shims.TalonEncoder;

public class Elevator extends SubsystemBase {

  private final WPI_TalonFX m_motor;
  private final PIDController m_elevatorPID;
  private final DutyCycleEncoder m_absoluteSpoolEncoder;     
  private final TalonEncoder m_elevatorMotorEncoder; 
  private final DigitalInput m_topLimitSwitch; 
  private final DigitalInput m_bottomLimitSwitch; 
  private double m_absEncoderZeroPositionTicks;
  private double clampLow = -ElevatorConstants.kMotorLimit; 
  private double clampHigh = ElevatorConstants.kMotorLimit;
  private boolean m_enabled; 

  public Elevator() {
    m_motor = MotorFactory.createTalonFX(ElevatorConstants.kMotorPort, Constants.kRioCAN);
    //m_motor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
    //m_motor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
    m_absoluteSpoolEncoder = new DutyCycleEncoder(ElevatorConstants.kAbsEncoderPort); 
    m_elevatorMotorEncoder = new TalonEncoder(m_motor); 
    m_elevatorMotorEncoder.setDistancePerPulse(ElevatorConstants.kDistPerMotorEncoderTick);
    m_elevatorPID = new PIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);  
    m_topLimitSwitch = new DigitalInput(ElevatorConstants.kTopLimitSwitchPort); 
    m_bottomLimitSwitch = new DigitalInput(ElevatorConstants.kBottomLimitSwitchPort); 
    //TODO: log, addDouble doesn't work. 
    //LogManager.addDouble("Elevator/error", () -> {return m_pid.getSetpoint() - getElevatorHeight();});
  }

  public void setMotorLimit(double powerLevel){
    clampLow = -powerLevel; 
    clampHigh = powerLevel; 
  }

  public void setMotorLimit(){
    setMotorLimit(ElevatorConstants.kMotorLimit);
  }

  @Override
  public void periodic() {
    /**
     * If we hit the bottom limit switch, then we must be at the zero position.
     * Thus set the setpoint to 0, then atSetpoint() will be triggered, causing the 
     * ResetEncoderAtBottom() command to end
     */
    
    if(m_enabled == true){
      double pid = m_elevatorPID.calculate(getElevatorHeightMeters());
      double pidClamped = MathUtil.clamp(pid, clampLow, clampHigh);
      m_motor.set(ControlMode.PercentOutput, pidClamped);   
    }
  }

  public void close() {
    // close the ports
    //TODO: Is there any way to close the ports of the limit switches attached to the motor? 
    m_absoluteSpoolEncoder.close();
  }

  public void set(double power){
    if(m_motor.get() > 0 && getBottomLimitSwitch() == true){
      m_motor.set(power); 
    }
  
    if(m_motor.get() <0 && getTopLimitSwitch() == true){
      m_motor.set(power); 
    }

    else{
      m_motor.set(0); 
    }
  }
  
  public void enableDisablePID(boolean condition){
    if(condition){
      m_enabled = true; 
    }
    if(condition == false){
      m_enabled = false; 
    }
  }
  public void disablePID(){
    m_enabled = false; 
  }

  public void setSetpointMeters(double setPointMeters){
    m_elevatorPID.setSetpoint(setPointMeters);
  }

  // public boolean getBottomLimitSwitch(){
  //  if(m_motor.getSensorCollection().isRevLimitSwitchClosed() == 0){
  //   return true; 
  //  }
  //   return false; 
  // }

  // public boolean getTopLimitSwitch(){
  //   if(m_motor.getSensorCollection().isFwdLimitSwitchClosed() == 0){
  //    return true; 
  //   }
  //    return false; 
  // }

  public boolean getBottomLimitSwitch(){
    return m_bottomLimitSwitch.get(); 
  }

  public boolean getTopLimitSwitch(){
    return m_topLimitSwitch.get(); 
  }

  public void resetMotorEncoder(){
    m_elevatorMotorEncoder.reset(); 
  }
 
  /**
   * Determine the elevator height from the motor encoder.
   * @return return height in meters
   */
  public double getElevatorHeightMeters() {
    return m_elevatorMotorEncoder.getDistance(); 
  }

  /**
   * 
   * Zero position is the value of the absolute encoder after the elevator
   * hits the bottom limit switch. 
   * @return return the absolute encoder's zero position in ticks. 
   * 
   */
  public double setAbsEncoderZeroPos(){
    m_absEncoderZeroPositionTicks = m_absoluteSpoolEncoder.getAbsolutePosition();
    return m_absEncoderZeroPositionTicks; 
  }

  public boolean atSetpoint() {
    return m_elevatorPID.atSetpoint();
  }

}

