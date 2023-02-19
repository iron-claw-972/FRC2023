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
import edu.wpi.first.math.controller.ElevatorFeedforward;
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
  private final ElevatorFeedforward m_elevatorFF;
  private final DutyCycleEncoder m_absEncoder;     
  private final TalonEncoder m_talonEncoder; 
  private final DigitalInput m_topLimitSwitch; 
  private final DigitalInput m_bottomLimitSwitch; 

  private double m_absEncoderZeroPositionTicks;
  private boolean m_enabled; 

  public Elevator() {
    m_motor = MotorFactory.createTalonFX(ElevatorConstants.kMotorPort, Constants.kRioCAN);
    //m_motor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
    //m_motor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
    m_absEncoder = new DutyCycleEncoder(ElevatorConstants.kAbsEncoderPort); 
    m_talonEncoder = new TalonEncoder(m_motor); 
    m_talonEncoder.setDistancePerPulse(ElevatorConstants.kDistPerPulse);
   
    m_elevatorPID = new PIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);
    m_elevatorFF = new ElevatorFeedforward(ElevatorConstants.kS, ElevatorConstants.kG, ElevatorConstants.kV, ElevatorConstants.kA);

    m_topLimitSwitch = new DigitalInput(ElevatorConstants.kTopLimitSwitchPort); 
    m_bottomLimitSwitch = new DigitalInput(ElevatorConstants.kBottomLimitSwitchPort); 
    
    //TODO: log, addDouble doesn't work. 
    //LogManager.addDouble("Elevator/error", () -> {return m_pid.getSetpoint() - getElevatorHeight();});
    
    m_motor.setSafetyEnabled(true);
    
  }

  @Override
  public void periodic() {
    /**
     * If we hit the bottom limit switch, then we must be at the zero position.
     * Thus set the setpoint to 0, then atSetpoint() will be triggered, causing the 
     * ResetEncoderAtBottom() command to end
     */
    
    if(m_enabled) {
      double pid = m_elevatorPID.calculate(getHeight());
      double ff = m_elevatorFF.calculate(ElevatorConstants.kVelocity, ElevatorConstants.kAccel); 

      double pidClamped = MathUtil.clamp(pid, -ElevatorConstants.kPowerLimit, ElevatorConstants.kPowerLimit);
      double finalMotorPower = pidClamped + ff; 
      
      set(finalMotorPower);
    } else {
      m_motor.feed();
    }
  }

  public void close() {
    // close the ports
    //TODO: Is there any way to close the ports of the limit switches attached to the motor? 
    m_topLimitSwitch.close(); 
    m_bottomLimitSwitch.close(); 
    m_absEncoder.close();
  }

  public void set(double power) {
    if((m_motor.get() > 0 && isBottom()) || (m_motor.get() < 0 && isTop())){
      m_motor.set(power); 
    } else {
      m_motor.set(0); 
    }
  }
  
  public void setEnabled(boolean isEnabled){
    m_enabled = isEnabled; 
  }


  public void setSetpoint(double setpoint){
    m_elevatorPID.setSetpoint(setpoint);
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

  public boolean isBottom(){
    if(m_bottomLimitSwitch.get() == false){
      return true; 
    } else{
      return false; 
    }
  }

  public boolean isTop(){
    if(m_topLimitSwitch.get() == false){
      return true; 
    } else{
      return false; 
    }  }

  public void resetTalonEncoder(){
    m_talonEncoder.reset(); 
  }
 
  /**
   * Determine the elevator height from the motor encoder.
   * @return return height in meters
   */
  public double getHeight() {
    return m_talonEncoder.getDistance(); 
  }

  /**
   * 
   * Zero position is the value of the absolute encoder after the elevator
   * hits the bottom limit switch. 
   * @return return the absolute encoder's zero position in ticks. 
   * 
   */
  public double setAbsEncoderZeroPos(){
    m_absEncoderZeroPositionTicks = m_absEncoder.getAbsolutePosition();
    return m_absEncoderZeroPositionTicks; 
  }

  public boolean atSetpoint() {
    return m_elevatorPID.atSetpoint();
  }

  public void setMotorPower(double power){
    m_motor.set(power);
  }

}

