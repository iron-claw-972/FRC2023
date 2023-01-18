package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.controls.Driver;
import frc.robot.subsystems.Drivetrain;

public class DefaultDriveCommand extends CommandBase {
    private final Drivetrain m_drive;

    public DefaultDriveCommand(Drivetrain drive) {
        this.m_drive = drive;

        addRequirements(drive);
    }

    @Override
    public void execute() {

        double xSpeed = Driver.getForwardTranslation();
        double ySpeed = Driver.getSideTranslation();
        double rot = Driver.getRotation();
    
        m_drive.drive(xSpeed, ySpeed, rot, true);
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.drive(0.0, 0.0, 0.0, false);
    }
}
