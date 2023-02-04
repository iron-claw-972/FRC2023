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
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.util.MotorFactory;

public class DeployingBar extends SubsystemBase {

  private final CANSparkMax m_motor1;
  private final RelativeEncoder m_encoder1;
  private double setpoint = Constants.deploybar.kmaxExtension;
  private SparkMaxPIDController m_pid;

  public DeployingBar() {
    m_motor1 = MotorFactory.createSparkMAXDefault(Constants.deploybar.kmotorId, MotorType.kBrushless);
    m_encoder1 = m_motor1.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 8192);
    m_pid = m_motor1.getPIDController();
    m_pid.setP(Constants.deploybar.kp);
    m_pid.setI(Constants.deploybar.ki);
    m_pid.setD(Constants.deploybar.kd);
    m_pid.setReference(setpoint, CANSparkMax.ControlType.kPosition);
    m_motor1.setIdleMode(IdleMode.kBrake);
  }

  public void setSetpoint(double target){
    setpoint = target;
    m_pid.setReference(setpoint, CANSparkMax.ControlType.kPosition);
  }
  
  public double getEncoderValue(){
    return m_encoder1.getPosition();
  }

  public void toggleCoast(){
    if(m_motor1.getIdleMode() == IdleMode.kBrake && m_encoder1.getPosition() >= 0.75*Constants.deploybar.kmaxExtension){
        m_motor1.setIdleMode(IdleMode.kCoast);
    }
  }

}
