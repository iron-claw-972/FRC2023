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

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;


public class Intake extends SubsystemBase {

  private final CANSparkMax m_leftMotor;
  private final CANSparkMax m_rightMotor;
  private final ShuffleboardTab m_intakeTab;
  private final double kConeThreshold = 420;
  private final double kCubeThreshold = 69;
  private final double kCubeTolerance = 5;
  private boolean m_hasCone = false;
  private boolean m_hasCube = false;
  private double m_startTime = 0;
  private final double kCubeTimeThreshold = 0; // seconds
  private final Rev2mDistanceSensor m_distSensor = new Rev2mDistanceSensor(Port.kOnboard);

  public Intake(ShuffleboardTab intakeTab) {
    m_leftMotor = new CANSparkMax(IntakeConstants.kLeftMotorPort, MotorType.kBrushless);
    m_rightMotor = new CANSparkMax(IntakeConstants.kRightMotorPort, MotorType.kBrushless);
    m_leftMotor.restoreFactoryDefaults();
    m_rightMotor.restoreFactoryDefaults();
    m_intakeTab = intakeTab;
    m_distSensor.setAutomaticMode(true);
    setupShuffleboard();
  }

  public void intake(double speed) {
<<<<<<< HEAD
    leftMotor.set(-speed);
    rightMotor.set(speed);
=======
    m_leftMotor.set(speed);
    m_rightMotor.set(speed);
>>>>>>> c9e3f046608a3a097d3562b1cdeb9000b0fc1552
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
    double range = m_distSensor.getRange();
    if (range == -1 || range > kCubeThreshold + kCubeTolerance) { // Empty intake
      m_startTime = Timer.getFPGATimestamp();
      m_hasCone = false;
      m_hasCube = false;
    } 
    else if (range < kConeThreshold) { // Cone
      m_hasCone = true;
      m_hasCube = false;
    } 
    else if (range > kConeThreshold && range < kCubeThreshold + kCubeTolerance){ // Cube
      if (Timer.getFPGATimestamp() + kCubeTimeThreshold >= m_startTime) {
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

  private void setupShuffleboard(){
    m_intakeTab.addDouble("Proximity", m_distSensor::getRange);
    m_intakeTab.addDouble("Timestamp", m_distSensor::getTimestamp);
  }
  
} 