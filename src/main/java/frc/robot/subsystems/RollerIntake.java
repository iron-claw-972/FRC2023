/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.IntakeConstants;
import frc.robot.util.LogManager;
import frc.robot.util.MotorFactory;


public class RollerIntake extends SubsystemBase {

  public enum IntakeMode {
    INTAKE_CUBE, OUTTAKE_CUBE, INTAKE_CONE, OUTTAKE_CONE, DISABLED
  }

  public enum IntakePiece {
    CUBE, CONE, NONE
  }

  private final WPI_TalonFX m_intakeMotor;
  private final ShuffleboardTab m_intakeTab;

  private IntakeMode m_mode;
  private IntakePiece m_heldPiece;

  private double m_power;

  public RollerIntake(ShuffleboardTab intakeTab) {
    m_intakeMotor = MotorFactory.createTalonFX(IntakeConstants.kIntakeMotorId, Constants.kCanivoreCAN);
    
    m_power = 0;
    m_mode = IntakeMode.DISABLED;
    // During auto, this doesn't really matter, so we can just set it to NONE
    // TODO: Auto commands run that pick up a game piece should set this to the correct value
    m_heldPiece = IntakePiece.NONE;
    m_intakeTab = intakeTab;

    setupShuffleboard();

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
        m_power = 0;       
    }
    setMotorPower(m_power);

    if (Constants.kLogging) {
      LogManager.addDouble("Intake Motor Current", m_intakeMotor.getStatorCurrent());
      LogManager.addDouble("Intake Power", m_power);
    }
  }

  private void setupShuffleboard() {
    m_intakeTab.addString("Intake Mode", () -> m_mode.name());
    m_intakeTab.addDouble("Intake Motor Current", () -> m_intakeMotor.getStatorCurrent());
    m_intakeTab.addDouble("Intake Power", () -> m_power);
    m_intakeTab.addString("Held Game Piece", () -> m_heldPiece.name());
  }
  
  public void updateLogs() {
    
  }
  
  public boolean containsGamePiece() {
    return m_heldPiece != IntakePiece.NONE;
  }

  public boolean containsCone() {
    return m_heldPiece == IntakePiece.CONE;
  }

  public boolean containsCube() {
    return m_heldPiece == IntakePiece.CUBE;
  }

  public WPI_TalonFX getIntakeMotor() {
    return m_intakeMotor;
  }

  public IntakePiece getHeldGamePiece() {
    return m_heldPiece;
  }

  public void setHeldGamePiece(IntakePiece m_type) {
    m_heldPiece = m_type;
  }
} 