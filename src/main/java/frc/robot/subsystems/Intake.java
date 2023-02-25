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
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;  


public class Intake extends SubsystemBase {

  private final CANSparkMax leftMotor;
  private final CANSparkMax rightMotor;

  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort); 
  private final ColorMatch m_colorMatcher = new ColorMatch();
  private final Color kYellowTarget = new Color(0.361, 0.524, 0.113);
  private final Color kPurpleTarget = new Color(0.102, 0, 0.204);

  public Intake() {
    leftMotor = new CANSparkMax(IntakeConstants.kLeftMotorPort, MotorType.kBrushless);
    rightMotor = new CANSparkMax(IntakeConstants.kRightMotorPort, MotorType.kBrushless);
    leftMotor.restoreFactoryDefaults();
    rightMotor.restoreFactoryDefaults();

    m_colorMatcher.addColorMatch(kPurpleTarget);
    m_colorMatcher.addColorMatch(kYellowTarget);  
  }

  public void intake(double speed) {
    leftMotor.set(-speed);
    rightMotor.set(speed);
  }

  public void stop() {
    leftMotor.set(0);
    rightMotor.set(0);
  }

  public String getHeldObject() {
    if (m_colorSensor.getProximity() <= IntakeConstants.kGamePieceProximity) {
      Color curr = m_colorSensor.getColor();
      ColorMatchResult res = m_colorMatcher.matchClosestColor(curr);
      if (res.color == kPurpleTarget) {
        return "Cube";
      }
      else if (res.color == kYellowTarget) {
        return "Cone";
      }
    }
    return "None";
  }
} 