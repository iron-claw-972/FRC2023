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
import frc.robot.util.GamePieceType;
import frc.robot.util.LogManager;
import frc.robot.util.MotorFactory;


public class RollerIntake extends SubsystemBase {

  public enum IntakeMode {
    INTAKE_CUBE, OUTTAKE_CUBE, INTAKE_CONE, OUTTAKE_CONE, DISABLED
  }

  private final WPI_TalonFX m_intakeMotor;
  private final ShuffleboardTab m_intakeTab;

  private IntakeMode m_mode;
  private GamePieceType m_heldPiece;

  private double m_power;

  public RollerIntake(ShuffleboardTab intakeTab) {
    m_intakeMotor = MotorFactory.createTalonFX(IntakeConstants.kIntakeMotorId, Constants.kRioCAN);
    m_intakeMotor.setNeutralMode(IntakeConstants.kNeutralMode);
    m_intakeMotor.enableVoltageCompensation(true);

    m_power = 0;
    m_mode = IntakeMode.DISABLED;
    m_heldPiece = GamePieceType.NONE;

    m_intakeTab = intakeTab;

    if (Constants.kUseTelemetry) setupShuffleboard();

  }

  public void setMode(IntakeMode mode) {
    m_mode = mode; 
  }

  private void setMotorPower(double power) {
    m_intakeMotor.set(power); 
  }

  @Override
  public void periodic() {

    switch (m_mode) {
      case INTAKE_CUBE:
        m_power = IntakeConstants.kIntakeCubePower;
        break;
      case OUTTAKE_CUBE: 
        m_power = IntakeConstants.kOuttakeCubePower;
        break;  
      case INTAKE_CONE: 
        m_power = IntakeConstants.kIntakeConePower;
        break;       
      case OUTTAKE_CONE: 
        m_power = IntakeConstants.kOuttakeConePower; 
        break;       
      case DISABLED: 
        m_power = 0;
        break;
    }
    setMotorPower(m_power);

    if (Constants.kLogging) {
      LogManager.addDouble("Intake/current", getCurrent());
      LogManager.addDouble("Intake/power", m_power);
    }
  }

  private void setupShuffleboard() {
    m_intakeTab.addString("Intake Mode", () -> m_mode.name());
    m_intakeTab.addDouble("Intake Motor Current", () -> getCurrent());
    m_intakeTab.addDouble("Intake Power", () -> m_power);
    m_intakeTab.addString("Held Game Piece", () -> m_heldPiece.name());
  }
  
  public boolean containsGamePiece() {
    return m_heldPiece != GamePieceType.NONE;
  }

  public boolean containsCone() {
    return m_heldPiece == GamePieceType.CONE;
  }

  public boolean containsCube() {
    return m_heldPiece == GamePieceType.CUBE;
  }

  /**
   * Returns the current supply current of the motor.
   * @return the supply current
   */
  public double getCurrent() {
    return Math.abs(m_intakeMotor.getSupplyCurrent());
  }

  public GamePieceType getHeldGamePiece() {
    return m_heldPiece;
  }

  public void setHeldGamePiece(GamePieceType m_type) {
    m_heldPiece = m_type;
  }
} 