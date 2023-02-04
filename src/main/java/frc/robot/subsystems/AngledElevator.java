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

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.util.MotorFactory;

public class AngledElevator extends SubsystemBase {

  WPI_TalonFX m_elevatorMotor; //do I define these are private final? 

  public AngledElevator() {

    this(MotorFactory.createTalonFX(Constants.elevator.kElevatorMotor, Constants.kRioCAN));

  }

  public AngledElevator(WPI_TalonFX elevatorMotor){

    m_elevatorMotor = elevatorMotor; 

  }


  public void resetEncoders(){
    m_elevatorMotor.setSelectedSensorPosition(0); 

  }


  public double getElevatorHeightInches() {
    
    double elevatorHeight = m_elevatorMotor.getSelectedSensorPosition()*Constants.elevator.kElevatorDistPerMotorEncoderTick;
    return elevatorHeight;  
      
  }


}

