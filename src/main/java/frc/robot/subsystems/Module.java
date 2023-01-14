package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Robot;

public abstract class Module {
  
  public static Module create(int driveMotorID, int steerMotorID, int encoderID, double steerOffset) {
    if (Robot.isReal()) {
      return new ModuleReal(driveMotorID, steerMotorID, encoderID, steerOffset);
    } else {
      // return new ModuleSim(driveMotorID, steerMotorID, encoderID, steerOffset);
    }
    return null;
  }

  // public Module(int driveMotorID, int steerMotorID, int encoderID, double steerOffset) {
    // this.driveMotorID = driveMotorID;
    // this.steerMotorID = steerMotorID;
    // this.encoderID = encoderID;
    // this.steerOffset = steerOffset;
  // }

  public abstract double getAngle();
  public abstract double getTurnFeedForward();
  public abstract double getTurnOutput();
  public abstract double getDriveVelocity();
  public abstract double getCharacterizationVelocity();
  public abstract void setDesiredState(SwerveModuleState state);

  public abstract void characterize(double volts);
  
}
