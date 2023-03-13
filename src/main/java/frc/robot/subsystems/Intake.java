/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.util.LogManager;


public class Intake extends SubsystemBase {

  private final CANSparkMax m_leftMotor;
  private final CANSparkMax m_rightMotor;
  private final ShuffleboardTab m_intakeTab;

  public Intake(ShuffleboardTab intakeTab) {
    m_leftMotor = new CANSparkMax(6, MotorType.kBrushless);
    m_rightMotor = new CANSparkMax(11, MotorType.kBrushless);

    configMotors();

    m_intakeTab = intakeTab;

    setupShuffleboard();
  }

  private void configMotors() {
    m_leftMotor.restoreFactoryDefaults();
    m_rightMotor.restoreFactoryDefaults();    

    m_leftMotor.setInverted(false);
    m_rightMotor.setInverted(true);

    m_leftMotor.setIdleMode(IdleMode.kBrake);
    m_rightMotor.setIdleMode(IdleMode.kBrake);

    m_leftMotor.enableVoltageCompensation(Constants.kRobotVoltage);
    m_rightMotor.enableVoltageCompensation(Constants.kRobotVoltage);

    // m_leftMotor.setSmartCurrentLimit(IntakeConstants.kMotorCurrentLimit);
    // m_rightMotor.setSmartCurrentLimit(IntakeConstants.kMotorCurrentLimit);
  }

  public void intake(double speed) {
    setLeft(speed);
    setRight(speed);
  }

  public void spinOuttake(double speed){
    setLeft(speed);
    setRight(-speed);
  }

  public void stopIntake() {
    setLeft(0);
    setRight(0);
  }

  public void setIdleMode(IdleMode idleMode) {
    m_leftMotor.setIdleMode(idleMode);
    m_rightMotor.setIdleMode(idleMode);
  }

  private void setLeft(double power) {
    if (Constants.kLogging) LogManager.addDouble("Intake/left power", power);
    m_leftMotor.set(power);
  }

  private void setRight(double power) {
    if (Constants.kLogging) LogManager.addDouble("Intake/right power", power);
    m_rightMotor.set(power);
  }

  private void setupShuffleboard() {
    if (Constants.kUseTelemetry) {
      m_intakeTab.addDouble("Left Output Current (A)", () -> m_leftMotor.getOutputCurrent());
      m_intakeTab.addDouble("Right Output Current (A)", () -> m_rightMotor.getOutputCurrent());
    }
  }
} 