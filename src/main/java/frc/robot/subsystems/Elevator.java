/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.util.MotorFactory;

public class Elevator extends SubsystemBase {

  private final WPI_TalonFX m_elevatorMotor; //do I define these are private final? 
  private final DigitalInput m_bottomLimitSwitch;
  private final DigitalInput m_topLimitSwitch; 
  private final PIDController m_elevatorPID; 
  private final DutyCycleEncoder m_absolutespoolEncoder; 

  

  public Elevator(){
    this(
      MotorFactory.createTalonFX(Constants.elevator.kElevatorMotor, Constants.kRioCAN),
      new DigitalInput(-1),
      new DigitalInput(-1),
      new DutyCycleEncoder(-1)
    );

  }

  public Elevator(WPI_TalonFX motor, DigitalInput bottomSwitch, DigitalInput topSwitch, DutyCycleEncoder absoluteSpoolEncoder){
    m_elevatorMotor = motor; 
    m_bottomLimitSwitch = bottomSwitch; 
    m_topLimitSwitch = topSwitch;
    m_absolutespoolEncoder = absoluteSpoolEncoder; 

    m_elevatorPID = new PIDController(Constants.elevator.kElevatorP, Constants.elevator.kElevatorI, Constants.elevator.kElevatorD);  
  }


  public void resetEncoders(){    
    m_elevatorMotor.setSelectedSensorPosition(0); 
  }

  public void calibrationBottomSwitchNotTripped(){
    while(m_bottomLimitSwitch.get() == false){
      m_elevatorMotor.set(-0.25);
    }
  }

  public void calibrationBottomSwitchTripped(){
    if(m_bottomLimitSwitch.get()){

      m_elevatorMotor.set(0); 
      resetEncoders(); 
      m_elevatorMotor.set(0.25); 

    }
  }


  public void calibrationSetMaxElevatorHeight(){
    if(m_topLimitSwitch.get()){

      Constants.elevator.kElevatorTopHeightInches = getElevatorHeightInches(); //TODO: Put this value on shuffleboard 
    }
  }

  public void setElevatorMotorSpeed(double speed){
    m_elevatorMotor.set(speed); 
  }

  public void stopMotorsIfLimitSwitchesTripped(){
    if (m_topLimitSwitch.get() || m_bottomLimitSwitch.get()){ //Is the right way of writing or? 
      m_elevatorMotor.set(0);
    }
  }

  public double returnHeightError(double elevatorHeightDesired){
    double error = elevatorHeightDesired-getElevatorHeightInches(); 
    return error; 
  }

  public double returnClampedElevatorPID(double elevatorHeightDesired){
    double pidValue = MathUtil.clamp(m_elevatorPID.calculate(getElevatorHeightInches(), elevatorHeightDesired ),-0.25,0.25);
       //TODO: Increase clamping range to make motor go faster if needed

    return pidValue; 
  }






  public double getElevatorHeightInches() {
    
    double elevatorHeight = m_elevatorMotor.getSelectedSensorPosition()*Constants.elevator.kElevatorDistPerMotorEncoderTick;
    return elevatorHeight;  
      
  }


}

