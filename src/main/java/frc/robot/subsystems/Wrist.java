package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.FalconConstants;
import frc.robot.constants.WristConstants;
import frc.robot.util.LogManager;
public class Wrist extends SubsystemBase {
  private final WPI_TalonFX m_motor;
  private final PIDController m_pid;
  private final DutyCycleEncoder m_absEncoder;
  private final ShuffleboardTab m_wristTab;
  private boolean m_FFenabled =false;
  private boolean m_enabled = true; 
  private final SingleJointedArmSim m_armSim =
    new SingleJointedArmSim(
      WristConstants.kGearBox, 
      WristConstants.kArmReduction,
      24.109,
      WristConstants.kArmLength,
      WristConstants.kMinAngleRads,
      WristConstants.kMaxAngleRads,
      true,
      VecBuilder.fill(2*Math.PI/FalconConstants.kResolution)
      );
      private final DutyCycleEncoderSim m_EncoderSim;

// Create a Mechanism2d display of an Arm with a fixed ArmTower and moving Arm.
  private final Mechanism2d m_mech2d = new Mechanism2d(60, 60);
  private final MechanismRoot2d m_armPivot = m_mech2d.getRoot("ArmPivot", 30, 30);
  private final MechanismLigament2d m_armTower =
    m_armPivot.append(new MechanismLigament2d("ArmTower", 30, -90));
  private final MechanismLigament2d m_arm =
    m_armPivot.append(
        new MechanismLigament2d(
            "Arm",
            30,
            Units.radiansToDegrees(m_armSim.getAngleRads()),
            6,
            new Color8Bit(Color.kYellow)));
  
  public Wrist(ShuffleboardTab wristTab) {
    // configure the motor
    m_motor = new WPI_TalonFX(WristConstants.kMotorID, Constants.kCanivoreCAN);
    m_motor.setNeutralMode(NeutralMode.Brake);
    m_motor.setInverted(false); 
    
    //SIM
    SmartDashboard.putData("Arm Sim", m_mech2d);
    // configure the encoder
    m_absEncoder = new DutyCycleEncoder(WristConstants.kAbsEncoderPort); 
    m_EncoderSim = new DutyCycleEncoderSim(m_absEncoder);

    // make the PID controller
    m_pid = new PIDController(WristConstants.kP, WristConstants.kI, WristConstants.kD);
    // set the PID controller's tolerance
    m_pid.setTolerance(WristConstants.kTolerance);
    // go to the initial position (use the class method)

    // TODO: restore stowed position
    // setArmSetpoint(ArmConstants.kStowedAbsEncoderPos);
    setArmSetpoint(WristConstants.kMinAngleRads);

    if (Constants.kUseTelemetry) {
      m_wristTab = wristTab;
      setupShuffleboardTab();
      SmartDashboard.putData("4 bar arm PID", m_pid); 
    }
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
    if(true) {
      
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

  public void setupShuffleboardTab() {
    m_wristTab.add("PID", m_pid);
  }
  public void simulationPeriodic() {
    // In this method, we update our simulation of what our arm is doing
    // First, we set our "inputs" (voltages)    
    System.out.println(m_motor.getMotorOutputVoltage() * RobotController.getBatteryVoltage());
    m_armSim.setInput(m_motor.getMotorOutputVoltage() * RobotController.getBatteryVoltage());

    // Next, we update it. The standard loop time is 20ms.
    m_armSim.update(0.02);

    m_EncoderSim.set(m_armSim.getAngleRads()/(Math.PI*2));

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_armSim.getCurrentDrawAmps()));
    m_arm.setAngle(Units.radiansToDegrees(m_armSim.getAngleRads()));
  }
  public double getMotorvoltage(){
    return m_motor.getMotorOutputVoltage();
  }
}

