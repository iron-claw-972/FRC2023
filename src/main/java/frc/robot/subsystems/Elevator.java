/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
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
  private final DigitalInput m_bottomLimitSwitch;
  private final DigitalInput m_topLimitSwitch;
  private final PIDController m_elevatorPID;
  private final DutyCycleEncoder m_absoluteSpoolEncoder;     
  private final TalonEncoder m_elevatorMotorEncoder; 
  private double m_absEncoderZeroPositionTicks;

  public Elevator() {
    m_motor = MotorFactory.createTalonFX(ElevatorConstants.kMotorPort, Constants.kRioCAN);
    m_motor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
    m_motor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
    m_bottomLimitSwitch = new DigitalInput(ElevatorConstants.kBottomLimitSwitchPort); 
    m_topLimitSwitch = new DigitalInput(ElevatorConstants.kTopLimitSwitchPort);
    m_absoluteSpoolEncoder = new DutyCycleEncoder(ElevatorConstants.kAbsEncoderPort); 
    m_elevatorMotorEncoder = new TalonEncoder(m_motor); 
    m_elevatorMotorEncoder.setDistancePerPulse(ElevatorConstants.kDistPerMotorEncoderTickMeters);
    m_elevatorPID = new PIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);  
  }

  @Override
  public void periodic() {
    m_motor.set(ControlMode.PercentOutput, m_elevatorPID.calculate(getElevatorHeightMeters())); 
  }

  public void close() {
    // close the ports
    m_bottomLimitSwitch.close();
    m_topLimitSwitch.close();
    m_absoluteSpoolEncoder.close();
  }

  public void set(double power){
    m_motor.set(power); 
  }

  public void setSepointMeters(double setPointMeters){
    m_elevatorPID.setSetpoint(setPointMeters);
  }


  public void stopMotor(){
    m_motor.set(0); 
  }

  public boolean getBottomLimitSwitch(){
    return m_bottomLimitSwitch.get(); 
  }

  public boolean getTopLimitSwitch(){
    return m_topLimitSwitch.get(); 
  }

  public void resetMotorEncoder(){
    m_elevatorMotorEncoder.reset(); 
  }

  public void stopMotorsIfLimitSwitchesTripped(double power){
    if (m_topLimitSwitch.get() && power>0){
      m_motor.set(0); 
    }
    if (m_topLimitSwitch.get() && power <0){
      //the semicolon makes the if statement do nothing
      ;//TODO: Maybe remove this if unncesary 
    }
    if (m_bottomLimitSwitch.get() && power<0){
      m_motor.set(0); 
    }
    if (m_bottomLimitSwitch.get() && power>0){
      //the semicolon makes the if statement do nothing
      ;//TODO: Maybe remove this if unncesary     
    }
  }
 

  public double getHeightError(double setpointMeters){
    double error = setpointMeters-getElevatorHeightMeters(); 
    return error; 
  }

  public double getElevatorHeightMeters() {
    double elevatorHeight =m_elevatorMotorEncoder.getDistance();
    return elevatorHeight;  
  }

  public double setAbsEncoderZeroPos(){
    m_absEncoderZeroPositionTicks = m_absoluteSpoolEncoder.getAbsolutePosition();
    return m_absEncoderZeroPositionTicks; 
  }

}

