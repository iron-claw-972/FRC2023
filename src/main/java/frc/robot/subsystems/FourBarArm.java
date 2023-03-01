package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ArmConstants;

public class FourBarArm extends SubsystemBase {
  private final CANSparkMax m_motor;
  private final PIDController m_pid;
  private final DutyCycleEncoder m_absEncoder;
  private boolean m_enabled = false;

  public FourBarArm() {
    // configure the motor
    m_motor = new CANSparkMax(ArmConstants.kMotorId, MotorType.kBrushless);
    m_motor.setIdleMode(IdleMode.kBrake);
    //m_motor.setInverted(true); 

    // configure the encoder
    // TODO: use a kConstant instead of the 8192
    m_absEncoder = new DutyCycleEncoder(ArmConstants.kAbsEncoderId); 

    // make the PID controller
    m_pid = new PIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD);
    // set the PID controller's tolerance
    m_pid.setTolerance(ArmConstants.kTolerance);
    // go to the initial position (use the class method)
    setArmSetpoint(ArmConstants.kStowedAbsEncoderPos);

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
    if(m_enabled) {
      
      // calculate the PID power level
      double pidPower = m_pid.calculate(getAbsEncoderPos(),MathUtil.clamp(m_pid.getSetpoint(),ArmConstants.kStowedAbsEncoderPos, ArmConstants.kMaxArmExtension));
      pidPower = MathUtil.clamp(pidPower, ArmConstants.kMinMotorPower,ArmConstants.kMaxMotorPower); 
      // calculate the feedforward power (nothing for now)
      double feedforwardPower = 0.0;

      // set the motor power
      setMotorPower(pidPower + feedforwardPower);
    }
  }

  /**
   * Whether the FourBarArm has reached its commanded position.
   * @returns true when position has been reached
   */
  public boolean reachedSetpoint() {
    return m_pid.atSetpoint();
  }

  public void setMotorPower(double power){
    m_motor.set(MathUtil.clamp(power, ArmConstants.kMinMotorPower, ArmConstants.kMaxMotorPower));
  }

  public void setEnabled(boolean enable)  {
    m_enabled = enable;
  }


  public double getAbsEncoderPos(){
    return m_absEncoder.getDistance(); 
  }
}
