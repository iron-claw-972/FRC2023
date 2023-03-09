package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public abstract class Module {

    public abstract int getModuleIndex();

    public abstract void setDriveFeedForwardValues(double kS, double kV);

    public abstract double getSteerFeedForwardKV();

    public abstract double getSteerFeedForwardKS();

    public abstract double getDriveFeedForwardKV();

    public abstract double getDriveFeedForwardKS();

    public abstract void stop();

    public abstract void setOptimize(boolean optimizeState);

    public abstract void enableStateDeadband(boolean stateDeadBand);

    public abstract void setDesiredState(SwerveModuleState swerveModuleState, boolean isOpenLoop);

    public abstract SwerveModulePosition getPosition();

    public abstract SwerveModuleState getState();

    public abstract void resetToAbsolute();

    public abstract void setAngle(Rotation2d rotation2d);

    public abstract void setDriveVoltage(double voltage);

    public abstract void setSteerVoltage(double voltage);

    public abstract double getSteerVelocity();

    public abstract double getDriveVelocityError();

    public abstract Rotation2d getAngle();

    public abstract double getDesiredVelocity();

    public abstract Rotation2d getDesiredAngle();
    
}