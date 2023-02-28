/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;


public class Intake extends SubsystemBase {

  private final CANSparkMax m_leftMotor;
  private final CANSparkMax m_rightMotor;
  private final ShuffleboardTab m_intakeTab;
  private final Rev2mDistanceSensor m_distSensor = new Rev2mDistanceSensor(Port.kOnboard);

  private boolean m_hasCone = false;
  private boolean m_hasCube = false;
  private double m_startTime = 0;

  private double m_range = -1;
  private double m_timestamp = -1;

  public Intake(ShuffleboardTab intakeTab) {
    m_leftMotor = new CANSparkMax(IntakeConstants.kLeftMotorPort, MotorType.kBrushless);
    m_rightMotor = new CANSparkMax(IntakeConstants.kRightMotorPort, MotorType.kBrushless);
    m_leftMotor.restoreFactoryDefaults();
    m_rightMotor.restoreFactoryDefaults();
    m_intakeTab = intakeTab;
    m_distSensor.setAutomaticMode(true);
    m_distSensor.setEnabled(true);
    setupShuffleboard();
  }

  public void intake(double speed) {
    m_leftMotor.set(speed);
    m_rightMotor.set(speed);
  }

  public boolean containsGamePiece(){
    return m_hasCone || m_hasCube;
  }

  public void stopIntake() {
      m_leftMotor.set(0);
      m_rightMotor.set(0);
  }

  @Override
  public void periodic() {

    if (m_distSensor.isRangeValid()) {
      double range = m_distSensor.getRange();

      // Empty intake
      if (range == -1 || range > IntakeConstants.kCubeDistanceThreshold + IntakeConstants.kCubeDistanceTolerance) { // Empty intake
        m_startTime = Timer.getFPGATimestamp();
        m_hasCone = false;
        m_hasCube = false;
      } 
      // Cone
      else if (range < IntakeConstants.kConeDistanceThreshold) { 
        m_hasCone = true;
        m_hasCube = false;
      } 
      // Cube
      else if (range > IntakeConstants.kConeDistanceThreshold && range < IntakeConstants.kCubeDistanceThreshold + IntakeConstants.kCubeDistanceTolerance){ // Cube
        if (Timer.getFPGATimestamp() + IntakeConstants.kCubeTimeThreshold >= m_startTime) {
          m_hasCone = false;
          m_hasCube = true;
        }
        else {
        m_startTime = Timer.getFPGATimestamp();
        m_hasCone = false;
        m_hasCube = false;
        }
      }
    }

    if (m_distSensor.isRangeValid()) {
      m_timestamp = m_distSensor.getTimestamp();
      m_range = m_distSensor.GetRange();
    }
  }

  public double getRange() {
    return m_range;
  }

  public double getTimestamp() {
    return m_timestamp;
  }



  private void setupShuffleboard() {
    m_intakeTab.addDouble("Proximity", this::getRange);
    m_intakeTab.addDouble("Timestamp", this::getTimestamp);
  }

}