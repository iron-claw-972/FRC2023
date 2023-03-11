package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.WristConstants;

public class Wrist extends SubsystemBase {
    private final WPI_TalonFX m_motor;
    private final DutyCycleEncoder m_absEncoder;
    private WristMode m_mode;
    private double m_desiredPower = 0;
    private double m_desiredPosition = 0;

    public Wrist() {
        m_motor = new WPI_TalonFX(WristConstants.kMotorID, Constants.kCanivoreCAN);
        configWristMotor();

        m_absEncoder = new DutyCycleEncoder(WristConstants.kAbsEncoderPort);
        configAbsEncoder();

        m_mode = WristMode.DISABLED;
    }

    private void configWristMotor() {
        m_motor.configFactoryDefault();
        m_motor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(
            WristConstants.kEnableCurrentLimit,
            WristConstants.kContinuousCurrentLimit,
            WristConstants.kPeakCurrentLimit,
            WristConstants.kPeakCurrentDuration
        ));
        m_motor.config_kP(0, WristConstants.kP);
        m_motor.config_kI(0, WristConstants.kI);
        m_motor.config_kD(0, WristConstants.kD);
        m_motor.config_kF(0, WristConstants.kF);
        m_motor.setInverted(WristConstants.kMotorInvert);
        m_motor.setNeutralMode(WristConstants.kNeutralMode);
        m_motor.configVoltageCompSaturation(Constants.kRobotVoltage);
        m_motor.enableVoltageCompensation(true);
    }

    private void configAbsEncoder() {
        m_absEncoder.setDistancePerRotation(1);
    }

    public void calibrateEncoder() {}
    
    public enum WristMode {
        DISABLED, MANUAL, POSITION
    }

    public void setMode(WristMode mode) {
        m_mode = mode;
    }

    public void setDesiredPower(double desiredPower) {
        m_desiredPower = desiredPower;
    }

    public void setDesiredPosition(double desiredPosition) {
        m_desiredPosition = desiredPosition;
    }

    @Override
    public void periodic() {
        switch(m_mode) {
            case DISABLED:
                m_motor.stopMotor();
                break;
            case MANUAL:
                m_motor.set(TalonFXControlMode.PercentOutput, m_desiredPower);
                break;
            case POSITION:
                m_motor.set(TalonFXControlMode.Position, m_desiredPosition, DemandType.ArbitraryFeedForward, 1.5 * Math.cos(0));
                break;
        }
    }
}
