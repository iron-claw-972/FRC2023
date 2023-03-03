/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.IntakeConstants;
import frc.robot.util.LogManager;


public class Intake extends SubsystemBase {

  private final CANSparkMax m_leftMotor;
  private final CANSparkMax m_rightMotor;
  private final ShuffleboardTab m_intakeTab;
  private final Rev2mDistanceSensor m_distSensor = new Rev2mDistanceSensor(Port.kOnboard);

  private boolean m_hasCone = false;
  private boolean m_hasCube = false;
  private double m_timeLastNotSeenCube = 0;

  private double m_range = -1;
  private double m_timestamp = -1;

  public Intake(ShuffleboardTab intakeTab) {
    m_leftMotor = new CANSparkMax(IntakeConstants.kLeftMotorPort, MotorType.kBrushless);
    m_rightMotor = new CANSparkMax(IntakeConstants.kRightMotorPort, MotorType.kBrushless);
    m_leftMotor.restoreFactoryDefaults();
    m_rightMotor.restoreFactoryDefaults();
    m_leftMotor.setSmartCurrentLimit(10);
    m_rightMotor.setSmartCurrentLimit(10);
    m_intakeTab = intakeTab;
    m_distSensor.setAutomaticMode(true);
    m_distSensor.setEnabled(true);
    setupShuffleboard();
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

  public boolean containsGamePiece(){
    return m_hasCone || m_hasCube;
  }

  private void setLeft(double power) {
    if (Constants.kLogging) LogManager.addDouble("Intake/left power", power);
    m_leftMotor.set(power);
  }

  private void setRight(double power) {
    if (Constants.kLogging) LogManager.addDouble("Intake/right power", power);
    m_rightMotor.set(power);
  }

  @Override
  public void periodic() {

    double range = m_distSensor.getRange();

    if (range == -1 || range > IntakeConstants.kCubeDistanceThreshold) { // Empty intake
      m_timeLastNotSeenCube = Timer.getFPGATimestamp();
      m_hasCone = false;
      m_hasCube = false;
    } 
    else if (range < IntakeConstants.kConeDistanceThreshold) { // Cone
      m_timeLastNotSeenCube = Timer.getFPGATimestamp();
      m_hasCone = true;
      m_hasCube = false;
    } 
    else if (range > IntakeConstants.kConeDistanceThreshold && range < IntakeConstants.kCubeDistanceThreshold){ // Cube
      if (Timer.getFPGATimestamp() - m_timeLastNotSeenCube > IntakeConstants.kCubeTimeThreshold) {
        m_hasCone = false;
        m_hasCube = true;
      } else {
        m_hasCone = false;
        m_hasCube = false;
      }
    }

    m_timestamp = m_distSensor.getTimestamp();
    m_range = m_distSensor.GetRange();

    if (Constants.kLogging) updateLogs();
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

  public void updateLogs(){
    LogManager.addBoolean("Intake/contains cube", m_hasCube);
    LogManager.addBoolean("Intake/contains cone", m_hasCone);
  }
  
} 