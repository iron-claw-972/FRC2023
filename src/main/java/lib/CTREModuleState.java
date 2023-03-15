package lib;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class CTREModuleState {
  
  /**
  * Minimize the change in heading the desired swerve module state would require by potentially
  * reversing the direction the wheel spins. Customized from WPILib's version to include placing
  * in appropriate scope for CTRE onboard control.
  *
  * @param desiredState The desired state.
  * @param currentAngle The current module angle.
  */
  public static SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
    double targetAngle = MathUtil.inputModulus(
      desiredState.angle.getRadians(), 
      currentAngle.getRadians() - Math.PI,
      currentAngle.getRadians() + Math.PI
    );
    double targetSpeed = desiredState.speedMetersPerSecond;
    
    if (Math.abs(targetAngle - currentAngle.getRadians()) > Math.PI/2) {
      targetAngle = MathUtil.inputModulus(
        desiredState.angle.getRadians(), 
        currentAngle.getRadians() - Math.PI/2,
        currentAngle.getRadians() + Math.PI/2
      );
      targetSpeed = -targetSpeed;
    }

    return new SwerveModuleState(targetSpeed, new Rotation2d(targetAngle));
  }
}
