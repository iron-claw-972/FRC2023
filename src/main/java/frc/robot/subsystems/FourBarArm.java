package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.Constants;
import frc.robot.util.LogManager;

public class FourBarArm extends SubsystemBase {
  private final CANSparkMax m_motor;
  private final PIDController m_pid;
  private final DutyCycleEncoder m_absEncoder;
  private boolean m_enabled = true;

  public FourBarArm() {
    // configure the motor
    m_motor = new CANSparkMax(ArmConstants.kMotorId, MotorType.kBrushless);
    m_motor.setIdleMode(IdleMode.kBrake);
    m_motor.setInverted(true); 

    // configure the encoder
    m_absEncoder = new DutyCycleEncoder(ArmConstants.kAbsEncoderId); 

    // make the PID controller
    m_pid = new PIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD);
    // set the PID controller's tolerance
    m_pid.setTolerance(ArmConstants.kTolerance);
    // go to the initial position (use the class method)

    // TODO: restore stowed position
    // setArmSetpoint(ArmConstants.kStowedAbsEncoderPos);
    setArmSetpoint(getAbsEncoderPos());

    SmartDashboard.putData("4 bar arm PID", m_pid); 
  }

  /**
   * Set the FourBarArm's desired position.
   * @param setpoint the desired arm position (in radians)
   */
  public void setArmSetpoint(double setpoint) {
    // set the PID integration error to zero.
    m_pid.reset();
    // set the PID desired position
    m_pid.setSetpoint(setpoint);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm Abs Encoder Value", getAbsEncoderPos());
    if(m_enabled) {
      
      // calculate the PID power level
      double pidPower = m_pid.calculate(getAbsEncoderPos(),MathUtil.clamp(m_pid.getSetpoint(),ArmConstants.kStowPos, ArmConstants.kMaxArmExtensionPos));
      // calculate the feedforward power (nothing for now)
      double feedforwardPower = 0.0;

      // set the motor power
      setMotorPower(pidPower + feedforwardPower);
    }

    if (Constants.kLogging) updateLogs();
  }

  /**
   * Whether the FourBarArm has reached its commanded position.
   * @returns true when position has been reached
   */
  public boolean reachedSetpoint() {
    return m_pid.atSetpoint();
  }

  public void setMotorPower(double power) {
    power = MathUtil.clamp(power, ArmConstants.kMinMotorPower, ArmConstants.kMaxMotorPower);
    m_motor.set(power);
    if (Constants.kLogging) LogManager.addDouble("Four Bar/motor power", power);
  }

  public void setEnabled(boolean enable)  {
    m_enabled = enable;
  }

  public double getAbsEncoderPos(){
    return m_absEncoder.getAbsolutePosition() * -1; 
  }

  public void updateLogs(){
    LogManager.addDouble("Four Bar/position", getAbsEncoderPos());
  }
}
