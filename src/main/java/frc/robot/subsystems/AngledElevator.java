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

private final WPI_TalonFX m_elevatorMotor;
private final Encoder m_encoder; 

  public AngledElevator() {

    this(MotorFactory.createTalonFX(Constants.elevator.kElevatorMotor, Constants.kRioCAN));

  }

  public AngledElevator(WPI_TalonFX elevatorMotor, Encoder encoder){

    m_elevatorMotor = elevatorMotor; 
    m_encoder = encoder; 

  }


  public void resetEncoders(){

    //TODO: Write code to reset encoders

  }


  public double getElevatorHeight() {
  
    //TODO: Write code to get elevator height
      
  }


}

