package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.arm.ExtendToPosition;
import frc.robot.constants.ArmConstants;
import lib.controllers.GameController;
import lib.controllers.GameController.Button;

public class FourBarArm extends SubsystemBase {
  private final CANSparkMax m_motor;
  private final PIDController m_pid;
  private final RelativeEncoder m_encoder;

  private static GameController operator = new GameController(0);

  private final SingleJointedArmSim armSim = 
  new SingleJointedArmSim(
    ArmConstants.armSimMotor, 
    ArmConstants.armReduction, 
    ArmConstants.armMOI, 
    ArmConstants.armLength, 
    Units.degreesToRadians(0), 
    Units.degreesToRadians(180), 
    ArmConstants.armMass, 
    true
    );
  private double armPositionDeg = 0;
  private double kArmEncoderDistPerPulse = 2.0*Math.PI/8192;
  private final Encoder dummyEncoder = new Encoder(0, 1);
  private final EncoderSim encoderSim = new EncoderSim(dummyEncoder);

  public static final String kArmPositionKey = "ArmPosition";
  public static final String kArmPKey = "ArmP";

  // constructor arguments TBD
  private final Mechanism2d mech2d = new Mechanism2d(60, 60);
  private final MechanismRoot2d armPivot = mech2d.getRoot("ArmPivot", 30, 30);
  private final MechanismLigament2d armTower =
    armPivot.append(new MechanismLigament2d("ArmTower", 30, -90));
  private final MechanismLigament2d armDiagram =
      armPivot.append(
          new MechanismLigament2d(
              "Arm",
              30,
              Units.radiansToDegrees(armSim.getAngleRads()),
              6,
              new Color8Bit(Color.kYellow)));

  public FourBarArm() {
    m_motor = new CANSparkMax(ArmConstants.motorID, MotorType.kBrushless);
    m_motor.setIdleMode(IdleMode.kBrake);

    m_encoder = m_motor.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 8192);
    m_encoder.setPositionConversionFactor(2*Math.PI);
    m_encoder.setVelocityConversionFactor(2*Math.PI/60);  

    m_pid = new PIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD);
    m_pid.setSetpoint(ArmConstants.initialPosition);
    m_pid.setTolerance(ArmConstants.kTolerance);

    dummyEncoder.setDistancePerPulse(kArmEncoderDistPerPulse);

    SmartDashboard.putData("Arm Sim", mech2d);
    armTower.setColor(new Color8Bit(Color.kBlue));

    operator.get(Button.B).onTrue(new InstantCommand(() -> setArmSetpoint(ArmConstants.topPosition)));
    operator.get(Button.X).onTrue(new InstantCommand(() -> setArmSetpoint(ArmConstants.middlePosiiton)));
    operator.get(Button.A).onTrue(new InstantCommand(() -> setArmSetpoint(ArmConstants.initialPosition)));
    operator.get(Button.Y).onTrue(new InstantCommand(() -> setArmSetpoint(ArmConstants.shelfPosition)));
  }

  @Override
  public void simulationPeriodic() {
    // In this method, we update our simulation of what our arm is doing
    // First, we set our "inputs" (voltages)
    armSim.setInput(m_motor.get() * RobotController.getBatteryVoltage());

    // Next, we update it. The standard loop time is 20ms.
    armSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    encoderSim.setDistance(armSim.getAngleRads());
    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(armSim.getCurrentDrawAmps()));

    // Update the Mechanism Arm angle based on the simulated arm angle
    armDiagram.setAngle(Units.radiansToDegrees(armSim.getAngleRads()));
  }

  public void close() {
    m_motor.close();
    dummyEncoder.close();
    mech2d.close();
    armPivot.close();
    m_pid.close();
    armDiagram.close();
  }

  public double getRadians() {
    if (RobotBase.isReal()) {
      return m_encoder.getPosition();
    }
    return dummyEncoder.getDistance();
  }

  public void setArmSetpoint(double setpoint) {
    m_pid.reset();
    m_pid.setSetpoint(setpoint);
  }

  @Override
  public void periodic() {
    m_motor.set(MathUtil.clamp(m_pid.calculate(getRadians()), ArmConstants.minMotorPower, ArmConstants.maxMotorPower));
  }

  public boolean reachedSetpoint() {
    return m_pid.atSetpoint();
  }
}
