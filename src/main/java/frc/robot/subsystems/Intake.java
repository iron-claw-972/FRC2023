/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.IntakeConstants;
import frc.robot.util.LogManager;
import frc.robot.util.MotorFactory;


public class Intake extends SubsystemBase {

  public enum IntakeMode {
    INTAKE_CUBE, OUTTAKE_CUBE, INTAKE_CONE, OUTTAKE_CONE, DISABLED
  }

  public enum IntakeObject {
    CUBE, CONE, NONE
  }

  private final WPI_TalonFX m_intakeMotor;
  private final ShuffleboardTab m_intakeTab;

  private IntakeMode m_mode;
  private IntakeObject m_object;

  private double m_power;

  public Intake(ShuffleboardTab intakeTab) {
    m_intakeMotor = MotorFactory.createTalonFX(IntakeConstants.kIntakeMotorId, Constants.kCanivoreCAN);
    configMotors();
    
    m_power = 0;
    m_mode = IntakeMode.DISABLED;
    // During auto, this doesn't really matter, so we can just set it to NONE
    m_object = IntakeObject.NONE;
    m_intakeTab = intakeTab;

    setupShuffleboard();
  
    if (Constants.kLogging) {
      LogManager.addDouble("Intake Motor Current", () -> m_intakeMotor.getStatorCurrent());
      LogManager.addDouble("Intake Power", () -> m_power);
    }

  }

  private void configMotors() {
    m_intakeMotor.setNeutralMode(IntakeConstants.kNeutralMode);
  }

  public void setIntakeMode(IntakeMode mode){
    m_mode = mode; 
  }

  private void setMotorPower(double power){
    m_intakeMotor.set(power); 
  }

  @Override
  public void periodic() {

    switch (m_mode) {
      case INTAKE_CUBE: 
        m_power = IntakeConstants.kIntakeCubePower;
      case OUTTAKE_CUBE: 
        m_power = IntakeConstants.kOuttakeCubePower;   
      case INTAKE_CONE: 
        m_power = IntakeConstants.kIntakeConePower;        
      case OUTTAKE_CONE: 
        m_power = IntakeConstants.kOuttakeConePower;        
      case DISABLED: 
        m_power = IntakeConstants.kStopPower;       
    }
    setMotorPower(m_power);
  }

  private void setupShuffleboard() {
    m_intakeTab.addString("Intake Mode", () -> m_mode.name());

  }
  
  public void updateLogs() {
    
  }
  
  public boolean hasObject() {
    return m_object != IntakeObject.NONE;
  }

  public boolean hasCone() {
    return m_object == IntakeObject.CONE;
  }

  public boolean hasCube() {
    return m_object == IntakeObject.CUBE;
  }

  public WPI_TalonFX getIntakeMotor() {
    return m_intakeMotor;
  }

  public IntakeObject getObject() {
    return m_object;
  }

  public void setObject(IntakeObject object) {
    m_object = object;
  }
} 