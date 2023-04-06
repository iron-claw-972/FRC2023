/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.Rev2mDistanceSensor.Unit;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.IntakeConstants;
import frc.robot.util.DrawMechanism;
import frc.robot.util.GamePieceType;
import frc.robot.util.LogManager;
import frc.robot.util.MotorFactory;


public class Intake extends SubsystemBase {

  public enum IntakeMode {
    INTAKE_CUBE, OUTTAKE_CUBE, OUTTAKE_CUBE_AUTO, INTAKE_CONE, OUTTAKE_CONE, DISABLED
  }

  private final WPI_TalonFX m_intakeMotor;
  private final ShuffleboardTab m_intakeTab;
  private final Rev2mDistanceSensor m_distSensor;

  private IntakeMode m_mode;
  private GamePieceType m_heldPiece;

  private double m_power;


  public Intake(ShuffleboardTab intakeTab) {
    m_intakeMotor = MotorFactory.createTalonFX(IntakeConstants.kIntakeMotorId, Constants.kRioCAN);
    m_intakeMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(
      IntakeConstants.kEnableCurrentLimit,
      IntakeConstants.kContinuousCurrentLimit,
      IntakeConstants.kPeakCurrentLimit,
      IntakeConstants.kPeakCurrentDuration
    ));
    m_intakeMotor.setNeutralMode(IntakeConstants.kNeutralMode);
    m_intakeMotor.enableVoltageCompensation(true);

    if (RobotBase.isReal()) {
      m_distSensor = new Rev2mDistanceSensor(Port.kMXP);
      m_distSensor.setDistanceUnits(Unit.kMillimeters);
      m_distSensor.setEnabled(true);
      m_distSensor.setAutomaticMode(true);
    } else {
      m_distSensor = null;
    }

    m_power = 0;
    m_mode = IntakeMode.DISABLED;
    m_heldPiece = GamePieceType.NONE;

    m_intakeTab = intakeTab;


    if (Constants.kUseTelemetry) setupShuffleboard();

  }

  public void setMode(IntakeMode mode) {
    m_mode = mode;
    if (RobotBase.isSimulation())
      DrawMechanism.getInstance().setIntakeStatus(m_mode);
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
      case OUTTAKE_CUBE_AUTO: 
        m_power = IntakeConstants.kOuttakeCubePowerAuto;
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
    m_intakeTab.addDouble("Cone distance from center", () -> getConePos());
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

  /**
   * @return The distance (meters) of the cone from the center of the intake as a double. Zero if no cone detected.
   */
  public double getConePos() {
    // Get the range in millimeters
    double range = -1;
    if (RobotBase.isReal()) {
      range = m_distSensor.getRange();
    }

    // Just assume center distance if it can't detect anything
    if (range == -1 || (range / 1000.0) > IntakeConstants.kMaxDistanceSensorRange) {
      return 0;
    }

    // Convert to meters and adjust to center offset
    return (range / 1000.0) - IntakeConstants.kCenterDist;
  }
} 