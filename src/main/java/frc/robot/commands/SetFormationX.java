// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

/** Add your docs here. */
public class SetFormationX extends CommandBase {
    Drivetrain m_drive;
    public SetFormationX(Drivetrain drive){
        this.m_drive=drive;
        addRequirements(drive);
    }
    public void execute(){
        m_drive.m_swerveModuleStates = new SwerveModuleState[] {
            new SwerveModuleState(0.01, new Rotation2d(Units.degreesToRadians(-45))),
            new SwerveModuleState(0.01, new Rotation2d(Units.degreesToRadians(45))),
            new SwerveModuleState(0.01, new Rotation2d(Units.degreesToRadians(-45))),
            new SwerveModuleState(0.01, new Rotation2d(Units.degreesToRadians(45)))
          };
          m_drive.setModuleStates(m_drive.m_swerveModuleStates);
    }
    
}
