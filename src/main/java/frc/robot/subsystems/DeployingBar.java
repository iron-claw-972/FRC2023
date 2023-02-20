/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.DeployingBarConstants;
import frc.robot.util.MotorFactory;
import lib.ctre_shims.TalonEncoder;

public class DeployingBar extends SubsystemBase {

  private final WPI_TalonFX m_motor;
  private final PIDController m_pid;
  private final TalonEncoder m_talonEncoder;
  private final DigitalInput m_topLimitSwitch;
  private final DigitalInput m_bottomLimitSwitch;
  private boolean isEnabled;

  public DeployingBar() {
    m_motor = MotorFactory.createTalonFX(DeployingBarConstants.kMotor, Constants.kRioCAN);
    m_motor.setNeutralMode(NeutralMode.Brake);
    m_motor.setSafetyEnabled(true);
    m_talonEncoder = new TalonEncoder(m_motor);
    m_talonEncoder.reset();
    m_talonEncoder.setDistancePerPulse(DeployingBarConstants.kTalonDistancePerPulse);
    m_pid = new PIDController(DeployingBarConstants.kP, DeployingBarConstants.kI, DeployingBarConstants.kD);
    m_pid.setTolerance(DeployingBarConstants.kTolerance);
    m_topLimitSwitch = new DigitalInput(DeployingBarConstants.kTopLimitSwitch);
    m_bottomLimitSwitch = new DigitalInput(DeployingBarConstants.kBottomLimitSwitch);
    isEnabled = false;
  }

  @Override
  public void periodic() {
    if(isEnabled){
        m_motor.set(ControlMode.PercentOutput, m_pid.calculate(m_talonEncoder.getDistance()));
    }
    else{
      m_motor.set(ControlMode.PercentOutput, 0);
    }
  }

  public void setSpeed(double speed) {
    m_motor.set(ControlMode.PercentOutput, speed);
  }

  public void setSetpoint(double setpoint){
    m_pid.reset();
    m_pid.setSetpoint(setpoint);
  }

  public void calibrateEncoder(){
    DeployingBarConstants.kDeployPos = m_talonEncoder.getDistance();
  }

  public void setEnable(boolean enableStatus){
    isEnabled = enableStatus;
  }

  public boolean atLimitSwitch(){
    return m_bottomLimitSwitch.get();
  }

  public boolean atSetpoint(){
    return m_pid.atSetpoint();
  }
}