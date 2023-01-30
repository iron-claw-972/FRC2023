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
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.controls.Driver;
import frc.robot.util.MotorFactory;
import lib.controllers.GameController.Axis;

public class Intake extends SubsystemBase {

  private final CANSparkMax leftMotor;
  private final CANSparkMax rightMotor;

  public Intake() {

    leftMotor = new CANSparkMax(17, MotorType.kBrushless);
    rightMotor = new CANSparkMax(1, MotorType.kBrushless);
    leftMotor.restoreFactoryDefaults();
    rightMotor.restoreFactoryDefaults();

  }

  public void intake(double speed) {
    leftMotor.set(-speed);
    rightMotor.set(speed);
  }

  public void outtake(double speed) {
    leftMotor.set(speed);
    rightMotor.set(-speed);
  }

  public void stop() {
    leftMotor.set(0);
    rightMotor.set(0);
  }
}
