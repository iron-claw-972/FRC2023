/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.util.MotorFactory;

public class DeployingBar extends SubsystemBase {

  private final TalonFX m_motor1;
  private double setpoint;
  private PIDController m_pid;
  private boolean isEnabled;

  public DeployingBar() {
    m_motor1 = MotorFactory.createTalonFX(Constants.deploybar.kmotorId, getName());
    m_pid = new PIDController(Constants.deploybar.kp, Constants.deploybar.ki, Constants.deploybar.kd);
    m_motor1.setNeutralMode(NeutralMode.Brake);
  }
  @Override
  public void periodic() {
    if(isEnabled){
      m_motor1.set(ControlMode.PercentOutput, m_pid.calculate(getEncoderValue(), setpoint));
    }
  }

  public void setSetpoint(double target){
    setpoint = target;
  }

  public boolean atSetpoint(){
    return m_pid.atSetpoint();
  }
  
  public double getEncoderValue(){
    return m_motor1.getSelectedSensorPosition();
  }

  public void setEnableStatus(boolean enableStatus){
    if(enableStatus)
      isEnabled = true;
    else
      isEnabled = false;
  }

  public void zeroEncoders(){
    m_motor1.setSelectedSensorPosition(0);
  }

  public void resetPID(){
    m_pid.reset();
  }

}
