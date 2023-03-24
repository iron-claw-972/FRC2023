package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotBase;
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
import frc.robot.util.MotorFactory;

public class Wrist extends SubsystemBase {
  private final WPI_TalonFX m_motor;
  private final PIDController m_pid;
  private final DutyCycleEncoder m_absEncoder;
  private final ShuffleboardTab m_wristTab;
  private boolean m_enabled = true; 
  private final DutyCycleEncoderSim m_encoderSim;
  private final SingleJointedArmSim m_armSim =
    new SingleJointedArmSim(
      WristConstants.kGearBox, 
      WristConstants.kArmReduction,
      WristConstants.kMOI,
      WristConstants.kArmLength,
      WristConstants.kMinAngleRads,
      WristConstants.kMaxAngleRads,
      true,
      VecBuilder.fill(2*Math.PI/FalconConstants.kResolution)
      );

// Create a Mechanism2d display of an Arm with a fixed ArmTower and moving Arm.
  private final Mechanism2d m_mech2d = new Mechanism2d(60, 60);
  private final MechanismRoot2d m_armPivot = m_mech2d.getRoot("ArmPivot", 30, 30);
  private final MechanismLigament2d m_armTower = m_armPivot.append(new MechanismLigament2d("ArmTower", -90, 0));
  private final MechanismLigament2d m_arm =
    m_armPivot.append(
        new MechanismLigament2d(
            "Arm",
            30,
            Units.radiansToDegrees(m_armSim.getAngleRads()),
            6,
            new Color8Bit(Color.kYellow)));
  
  public Wrist(ShuffleboardTab wristTab) {
    // configure the motor.
    m_motor = MotorFactory.createTalonFXSupplyLimit(WristConstants.kMotorID, Constants.kRioCAN, WristConstants.kContinuousCurrentLimit, WristConstants.kPeakCurrentLimit, WristConstants.kPeakCurrentDuration);
    m_motor.setNeutralMode(WristConstants.kNeutralMode);
    m_motor.setInverted(WristConstants.kMotorInvert); 
    
    //SIM
    SmartDashboard.putData("Arm Sim", m_mech2d);
    // configure the encoder
    m_absEncoder = new DutyCycleEncoder(WristConstants.kAbsEncoderPort); 
    m_absEncoder.setPositionOffset(WristConstants.kEncoderOffset);
    m_absEncoder.setDistancePerRotation(2*Math.PI);
    m_encoderSim = new DutyCycleEncoderSim(m_absEncoder);

    // make the PID controller
    m_pid = new PIDController(WristConstants.kP, WristConstants.kI, WristConstants.kD);
    // set the PID controller's tolerance
    m_pid.setTolerance(WristConstants.kTolerance);
    // go to the initial position (use the class method)

    setSetpoint(WristConstants.kStowPos);

    if (Constants.kUseTelemetry) {
      m_wristTab = wristTab;
      setupShuffleboardTab();
    }
  }

  /**
   * Set the Wrist's desired position.
   * @param setpoint the desired arm position (in rotations)
   */
  public void setSetpoint(double setpoint) {
    // set the PID integration error to zero.
    m_pid.reset();
    // set the PID desired position
    m_pid.setSetpoint(setpoint);
  }

  @Override
  public void periodic() {
    if(m_enabled) {
      // calculate the PID power level
      double pidPower = m_pid.calculate(!RobotBase.isSimulation()? getAbsEncoderPos(): m_armSim.getAngleRads(), MathUtil.clamp(m_pid.getSetpoint(), WristConstants.kMinAngleRads, WristConstants.kMaxAngleRads));
      if (Constants.kLogging) LogManager.addDouble("Wrist/pidOutput", pidPower);
      if (Constants.kUseTelemetry) SmartDashboard.putNumber("wrist pid output", pidPower);
      // calculate the value of kGravityCompensation
      double feedforwardPower = WristConstants.kGravityCompensation*Math.cos(getAbsEncoderPos());
      // set the motor power
      setMotorPower(pidPower+feedforwardPower);
    }

    if (Constants.kLogging) updateLogs();
  }

  /**
   * Whether the wrist has reached its commanded position.
   * @returns true when position has been reached
   */
  public boolean reachedSetpoint() {
    return m_pid.atSetpoint();
  }

  public void setMotorPower(double power) {
    power = MathUtil.clamp(power, -WristConstants.kMotorPowerClamp, WristConstants.kMotorPowerClamp);
    
    if (getAbsEncoderPos() <= WristConstants.kMinPos && power < 0 && !RobotBase.isSimulation()) {
      power = 0;
    }
    if (getAbsEncoderPos() >= WristConstants.kMaxPos && power > 0 && !RobotBase.isSimulation()) {
      power = 0;
    }
    
    m_motor.set(power);
    if (Constants.kLogging) LogManager.addDouble("Wrist/motor power", power);
    if (Constants.kUseTelemetry) SmartDashboard.putNumber("wrist power final", power);
  }

  public void setEnabled(boolean enable) {
    m_enabled = enable;
  }

  public double getAbsEncoderPos() {
    // inverted to make rotating towards stow positive
    // offset makes flat, facing out, zero
    return -m_absEncoder.getAbsolutePosition() + WristConstants.kEncoderOffset; 
  }

  public void updateLogs() {
    LogManager.addDouble("Wrist/position", getAbsEncoderPos());
  }

  public void setupShuffleboardTab() {
    m_wristTab.addNumber("Wrist Position", () -> getAbsEncoderPos());
    m_wristTab.add("wrist PID", m_pid);
  }

  public void simulationPeriodic() {

    // First, we set our "inputs" (voltages)    
    m_armSim.setInput(m_motor.get() * RobotController.getBatteryVoltage());

    // Next, we update it. The standard loop time is 20ms.
    m_armSim.update(0.02);
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_armSim.getCurrentDrawAmps()));
    m_encoderSim.setDistance(m_armSim.getAngleRads());
    m_arm.setAngle(Units.radiansToDegrees(getSimRads()));
  }

  public double getMotorvoltage(){
    return m_motor.getMotorOutputVoltage();
  }
  public double getSimRads(){
    return m_armSim.getAngleRads();
  }
}

