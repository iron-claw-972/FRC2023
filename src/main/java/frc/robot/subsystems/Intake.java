/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.Rev2mDistanceSensor.Port;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.Constants;
import frc.robot.constants.IntakeConstants;
import frc.robot.util.LogManager;
import frc.robot.util.MotorFactory;


public class Intake extends SubsystemBase {

  private final WPI_TalonFX m_intakeMotor;
  private IntakeMode m_mode; 


  public Intake(ShuffleboardTab intakeTab) {
    m_intakeMotor = MotorFactory.createTalonFX(IntakeConstants.kIntakeMotorId, Constants.kCanivoreCAN);
    configMotors();
    m_mode = IntakeMode.DISABLED;

  }


  private void configMotors() {
    //m_intakeMotor.setInverted(true); 
  }

  public enum IntakeMode {
    INTAKE_CUBE, OUTTAKE_CUBE, INTAKE_CONE, OUTTAKE_CONE, DISABLED
  }

  public void setIntakeMode(IntakeMode mode){
    m_mode = mode; 
  }

  public void setMotorPower(double power){
    m_intakeMotor.set(power); 
  }

  @Override
  public void periodic() {
    switch(m_mode){
      case INTAKE_CUBE: 
        setMotorPower(IntakeConstants.kIntakeCubePower); 
      case OUTTAKE_CUBE: 
        setMotorPower(IntakeConstants.kOuttakeCubePower);   
      case INTAKE_CONE: 
        setMotorPower(IntakeConstants.kIntakeConePower);        
      case OUTTAKE_CONE: 
        setMotorPower(IntakeConstants.kOuttakeConePower);        
      case DISABLED: 
        setMotorPower(IntakeConstants.kStopPower);       
      }
    }

} 