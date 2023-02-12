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
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.controls.Operator;
import frc.robot.util.MotorFactory;

public class Elevator extends SubsystemBase {

  private final WPI_TalonFX m_elevatorMotor; //do I define these are private final? 
  private final DigitalInput m_bottomLimitSwitch;
  private final DigitalInput m_topLimitSwitch; 
  private final PIDController m_elevatorPID; 
  private final DutyCycleEncoder m_absoluteSpoolEncoder; 

  public Elevator() {
    this(
      MotorFactory.createTalonFX(ElevatorConstants.kElevatorMotor, Constants.kRioCAN),
      new DigitalInput(-1),
      new DigitalInput(-1),
      new DutyCycleEncoder(-1)
    );
  }

  public Elevator(WPI_TalonFX motor, DigitalInput bottomSwitch, DigitalInput topSwitch, DutyCycleEncoder absoluteSpoolEncoder) {
    m_elevatorMotor = motor; 
    m_bottomLimitSwitch = bottomSwitch; 
    m_topLimitSwitch = topSwitch;
    m_absoluteSpoolEncoder = absoluteSpoolEncoder; 

    m_elevatorPID = new PIDController(ElevatorConstants.kElevatorP, ElevatorConstants.kElevatorI, ElevatorConstants.kElevatorD);  
  }

  public void set(double power){
    m_elevatorMotor.set(power); 
  }

  public void stopMotor(){
    m_elevatorMotor.set(0); 
  }

  public boolean returnBottomLimSwitchCondition(){
    return m_bottomLimitSwitch.get(); 
  }

  public boolean returnTopLimSwitchCondition(){
    return m_topLimitSwitch.get(); 
  }

  public void resetMotorEncoder(){
    m_elevatorMotor.setSelectedSensorPosition(0); 
  }

  public void stopMotorsIfLimitSwitchesTripped(double power){
    if (m_topLimitSwitch.get() && power>0){
      m_elevatorMotor.set(0); 
    }
    if (m_topLimitSwitch.get() && power <0){
      //the semicolon makes the if statement do nothing
      ;//TODO: Maybe remove this if unncesary 
    }
    if (m_bottomLimitSwitch.get() && power<0){
      m_elevatorMotor.set(0); 
    }
    if (m_bottomLimitSwitch.get() && power>0){
      //the semicolon makes the if statement do nothing
      ;//TODO: Maybe remove this if unncesary     
    }
  }
 

  public double returnHeightError(double elevatorHeightDesired){
    double error = elevatorHeightDesired-getElevatorHeightMeters(); 
    return error; 
  }

  public double returnClampedElevatorPID(double elevatorHeightDesired){
    double pidValue = MathUtil.clamp(m_elevatorPID.calculate(getElevatorHeightMeters(), elevatorHeightDesired ),-0.25,0.25);
       //TODO: Increase clamping range to make motor go faster if needed

    return pidValue; 
  }

  public double getElevatorHeightMeters() {
    double elevatorHeight = m_elevatorMotor.getSelectedSensorPosition()*ElevatorConstants.kElevatorDistPerMotorEncoderTickMeters;
    return elevatorHeight;  
  }

  public void calibrationZeroAbsEncoder() {
    ElevatorConstants.kElevatorAbsEncoderZeroPositionMeters = m_absoluteSpoolEncoder.getAbsolutePosition()*ElevatorConstants.kSpoolAbsEncoderDistancePerTickMeters; //TODO: CHECK IF THIS IS IN TICKS
  }

}

