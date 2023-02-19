package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ArmConstants;

public class FourBarArm extends SubsystemBase {
  private final CANSparkMax m_motor;
  private final PIDController m_pid;
  private final RelativeEncoder m_encoder;
  private boolean m_enabled = false;

  public FourBarArm() {
    // configure the motor
    m_motor = new CANSparkMax(ArmConstants.kmotorID, MotorType.kBrushless);
    m_motor.setIdleMode(IdleMode.kBrake);

    // configure the encoder
    // TODO: use a kConstant instead of the 8192
    m_encoder = m_motor.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 8192);
    // The RelativeEncoder reports angles in native revolutions by default.
    // See https://codedocs.revrobotics.com/java/com/revrobotics/relativeencoder
    // Change the encoder's reported value to radians (1 revolution = 2 pi radians).
    m_encoder.setPositionConversionFactor(2*Math.PI);
    // The RelativeEncoder reports RPM by default.
    // Change the velocity to radians per second (1 RPM = 2 pi radians / 60 seconds)
    m_encoder.setVelocityConversionFactor(2.0 * Math.PI / 60.0);  

 
    // make the PID controller
    m_pid = new PIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD);
    // set the PID controller's tolerance
    m_pid.setTolerance(ArmConstants.kTolerance);
    // go to the initial position (use the class method)
    setArmSetpoint(ArmConstants.kinitialPosition);
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
      double pidPower = m_pid.calculate(m_encoder.getPosition());
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
}
