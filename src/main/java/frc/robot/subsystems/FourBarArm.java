package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class FourBarArm extends SubsystemBase {
  private final CANSparkMax m_motor = new CANSparkMax(Constants.arm.motorID, MotorType.kBrushless);
  private final Encoder encoder = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
  private final PIDController pid = new PIDController(Constants.arm.kP, Constants.arm.kI, Constants.arm.kD);
  private int armSetpoint = Constants.arm.initialPosition;
  public void setSetpoint(int target) {
    armSetpoint = target;
  }
  public int getSetpoint() {
    return armSetpoint;
  }
  public int getEncoderPosition() {
    return encoder.get();
  }
  public PIDController getPID() {
    return pid;
  }
  public void setMotor(double speed) {
    m_motor.set(speed);
  }
}
