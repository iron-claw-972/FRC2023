package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.FalconConstants;
import frc.robot.constants.WristConstants;
import frc.robot.util.DrawMechanism;
import frc.robot.util.LogManager;
import frc.robot.util.MotorFactory;

public class Wrist extends SubsystemBase {
  private final WPI_TalonFX m_motor;
  final PIDController m_pid;

  final DutyCycleEncoder m_absEncoder;
  DutyCycleEncoderSim m_absEncoderSim;

  private final ShuffleboardTab m_wristTab;
  private boolean m_enabled = true;
  double m_pidPower = 0;

  /** Physics Simulator for the wrist. takes in a motor voltage and calculates how much the arm will move. */
  private SingleJointedArmSim m_armSim;
  
  public Wrist(ShuffleboardTab wristTab) {
    // configure the motor.
    m_motor = MotorFactory.createTalonFXSupplyLimit(
      WristConstants.kMotorID, 
      Constants.kRioCAN, 
      WristConstants.kContinuousCurrentLimit, 
      WristConstants.kPeakCurrentLimit, 
      WristConstants.kPeakCurrentDuration);
    m_motor.setNeutralMode(WristConstants.kNeutralMode);
    m_motor.setInverted(WristConstants.kMotorInvert); 

    m_wristTab = wristTab;

    // configure the encoder
    m_absEncoder = new DutyCycleEncoder(WristConstants.kAbsEncoderPort);
    // Cleaner encoder implementation
    // offset to zero (arm horizontal)
    m_absEncoder.setPositionOffset(WristConstants.kEncoderOffset);
    // scale to radians and invert direction
    m_absEncoder.setDistancePerRotation(-2.0 * Math.PI);

    // make the PID controller
    m_pid = new PIDController(WristConstants.kP, WristConstants.kI, WristConstants.kD);
    // set the PID controller's tolerance
    m_pid.setTolerance(WristConstants.kTolerance);

    // go to the initial position
    setSetpoint(WristConstants.kStowPos);

    if (Constants.kUseTelemetry) {
      setupShuffleboardTab();
    }

    // if a simulation, set up the simulation resources
    if (RobotBase.isSimulation()) {
      // make the simulator
      m_armSim = new SingleJointedArmSim(
        // to know how much the arm will move with a certain power, needs to know the motor, gear ratio, MOI, and length
        WristConstants.kGearBox, 
        WristConstants.kGearRatio,
        WristConstants.kMomentOfInertia,
        WristConstants.kLength,
        // prevents moving past min/max
        WristConstants.kMinPos,
        WristConstants.kMaxPos,
        true, // will simulate gravity
        // TODO: this calculation is wrong: DutyCycleEncoder has resolution of 1 Revolution / 1024.
        VecBuilder.fill(2*Math.PI / FalconConstants.kResolution) // a deviation of one motor tick in the angle
      );

      // make the encoder simulator
      // this allows us to set the encoder...
      m_absEncoderSim = new DutyCycleEncoderSim(m_absEncoder);
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
    if (m_enabled) {
      // obtain the wrist position
      double position = getAbsEncoderPos();

      // calculate the PID power level
      // for safety, clamp the setpoint to prevent tuning with SmartDashboard/Shuffleboard from commanding out of range
      // This continually changes the setpoint.
      // TODO: m_pidPower should be a local; logging and Shuffleboard should use m_motor.get()
      m_pidPower = m_pid.calculate(position, MathUtil.clamp(m_pid.getSetpoint(), WristConstants.kMinPos, WristConstants.kMaxPos));
      
      // calculate the value of kGravityCompensation
      double feedforwardPower = WristConstants.kGravityCompensation * Math.cos(position);

      // set the motor power
      setMotorPower(m_pidPower + feedforwardPower);
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

  /**
   * Sets the motor power, clamping it and ensuring it will not activate below/above the min/max positions
   */
  public void setMotorPower(double power) {
    // clamp power to a safe range
    power = MathUtil.clamp(power, -WristConstants.kMotorPowerClamp, WristConstants.kMotorPowerClamp);
    
    // TODO: Is this code needed? PID is only caller, and it should set reasonable values.
    if (getAbsEncoderPos() <= WristConstants.kMinPos && power < 0) {
      power = 0;
    }
    if (getAbsEncoderPos() >= WristConstants.kMaxPos && power > 0) {
      power = 0;
    }
    
    m_motor.set(power);
  }

  public void setEnabled(boolean enable) {
    m_enabled = enable;
  }

  /**
   * @return the absolute encoder position in rotations, zero being facing forward
   */
  public double getAbsEncoderPos() {
    // This is the wrong way to simulate the encoder. Should just use m_absEncoder.getDistance()
    // if (RobotBase.isSimulation()) return m_armSim.getAngleRads();
    // inverted to make rotating towards stow positive
    // offset makes flat, facing out, zero
    // return (-m_absEncoder.getAbsolutePosition() + WristConstants.kEncoderOffset) * 2 * Math.PI; 

    return m_absEncoder.getDistance();
  }

  public void updateLogs() {
    LogManager.addDouble("Wrist/position", getAbsEncoderPos());
    LogManager.addDouble("Wrist/motor power", m_motor.get());
    LogManager.addDouble("Wrist/pidOutput", m_pidPower);
  }

  public void setupShuffleboardTab() {
    m_wristTab.addNumber("Wrist Position", () -> getAbsEncoderPos());
    m_wristTab.add("wrist PID", m_pid);
    m_wristTab.addNumber("wrist power final", () -> m_motor.get());
    m_wristTab.addNumber("Wrist PID output", () -> m_pidPower);
    m_wristTab.addNumber("Wrist Error", () -> m_pid.getSetpoint() - getAbsEncoderPos());
  }

  public void simulationPeriodic() {
    // First, we set our "inputs" (the motor voltage)    
    m_armSim.setInput(m_motor.get() * RobotController.getBatteryVoltage());

    // update the physics simulation, telling it how much time has passed, and it will calculate how much the wrist has moved
    m_armSim.update(Constants.kLoopTime);

    // Cleaner encoder implementation: we know the angle now, so set the encoder to that value
    // TODO: ARGH! .setDistance() is setting rotations, so use .set()
    // m_absEncoderSim.setDistance(m_armSim.getAngleRads());
    m_absEncoderSim.set(m_armSim.getAngleRads() / m_absEncoder.getDistancePerRotation());

    // calculate the battery voltage based on the theoretical drawn amps.
    // TODO: this is the wrong way to calculate battery voltage; it needs the sum of all currents.
    // RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(m_armSim.getCurrentDrawAmps()));

    // update the drawing of the robot
    DrawMechanism.getInstance().setWristAngle(m_armSim.getAngleRads());
  }

  /**
   * Deallocate resources.
   * <p>
   * Test routines need to deallocate simulation resources.
   */
  public void close() {
    m_motor.close();
    m_absEncoder.close();
  }
}
