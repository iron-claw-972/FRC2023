package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.WristConstants;
import frc.robot.util.LogManager;
public class Wrist extends SubsystemBase {
    private final WPI_TalonFX m_motor;
    private final PIDController m_pid;
    private final DutyCycleEncoder m_absEncoder;
    private boolean m_FFenabled =false;
    private boolean m_enabled = true;
    public Wrist() {
    // configure the motor
    m_motor = new WPI_TalonFX(WristConstants.kMotorID, Constants.kCanivoreCAN);
    m_motor.setNeutralMode(NeutralMode.Brake);
    m_motor.setInverted(false); 

    // configure the encoder
    m_absEncoder = new DutyCycleEncoder(WristConstants.kAbsEncoderPort); 
    
    // make the PID controller
    m_pid = new PIDController(WristConstants.kP, WristConstants.kI, WristConstants.kD);
    // set the PID controller's tolerance
    m_pid.setTolerance(WristConstants.kTolerance);
    // go to the initial position (use the class method)

    // TODO: restore stowed position
    // setArmSetpoint(ArmConstants.kStowedAbsEncoderPos);
    setArmSetpoint(WristConstants.kStowPos);

    if (Constants.kUseTelemetry) SmartDashboard.putData("4 bar arm PID", m_pid); 
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
    if (Constants.kUseTelemetry) SmartDashboard.putNumber("Arm Abs Encoder Value", getAbsEncoderPos());
    if(m_enabled) {
      
      // calculate the PID power level
      double pidPower = m_pid.calculate(getAbsEncoderPos(), MathUtil.clamp(m_pid.getSetpoint(), WristConstants.kMaxArmExtensionPos, WristConstants.kStowPos));
      if (Constants.kLogging) LogManager.addDouble("Four Bar/pidOutput", pidPower);
      // calculate the value of kGravityCompensation
      double feedforwardPower = WristConstants.kGravityCompensation*Math.cos(getAbsEncoderPos()*Math.PI*2);

      // set the motor power
      setMotorPower(pidPower + (m_FFenabled ? feedforwardPower:0));
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
    power = MathUtil.clamp(power, WristConstants.kMinMotorPower, WristConstants.kMaxMotorPower);
    m_motor.set(power);
    if (Constants.kLogging) LogManager.addDouble("Four Bar/motor power", power);
  }

  public void setEnabled(boolean enable)  {
    m_enabled = enable;
  }

  public double getAbsEncoderPos(){
    return m_absEncoder.getAbsolutePosition(); 
  }

  public void updateLogs(){
    LogManager.addDouble("Four Bar/position", getAbsEncoderPos());
  }
}
