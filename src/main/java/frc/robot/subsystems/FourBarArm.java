package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class FourBarArm extends SubsystemBase {
  public final CANSparkMax m_motor;
  public final PIDController m_pid;
  public final RelativeEncoder m_encoder;

  private final SingleJointedArmSim m_armSim = 
    new SingleJointedArmSim(
      Constants.arm.armSimMotor, 
      Constants.arm.armReduction, 
      Constants.arm.armMOI, 
      Constants.arm.armLength, 
      Units.degreesToRadians(0), 
      Units.degreesToRadians(180), 
      Constants.arm.armMass, 
      true
      );
  // find way to use encoder sim with relative encoder type
  private final EncoderSim m_encoderSim = new EncoderSim(m_encoder);
  private double armPositionDeg = 0;
  private double kArmEncoderDistPerPulse = 2.0*Math.PI/8192;

  public FourBarArm() {
    m_motor = new CANSparkMax(Constants.arm.motorID, MotorType.kBrushless);
    m_encoder = m_motor.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 8192);
    m_pid = new PIDController(Constants.arm.kP, Constants.arm.kI, Constants.arm.kD);
  }
}
