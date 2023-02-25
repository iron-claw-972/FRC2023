package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.constants.swerve.DriveConstants;
import frc.robot.subsystems.Drivetrain;

public class BalanceCommand extends CommandBase {

    private Drivetrain m_drive;
    private final PIDController m_pid;

    private double m_currentAngle, m_output;

    private boolean m_usePitch;

    private int m_inverted;
 
    public BalanceCommand(Drivetrain drive) {
        m_drive = drive;
        addRequirements(drive);
        m_pid = new PIDController(DriveConstants.kBalanceP, DriveConstants.kBalanceI, DriveConstants.kBalanceD);
        m_pid.setTolerance(DriveConstants.kBalanceTolerance);
    }

    @Override
    public void initialize() {
        m_inverted = 1;

        m_pid.setSetpoint(0);
        if(Math.abs(Math.PI/2 - Math.abs(m_drive.getAngleHeading())) > Math.PI/4)  { //Determines whether to use roll or pitch
            m_usePitch = false;
        }
        else  {
            m_usePitch = true;
        }
        if(m_drive.getAngleHeading() < 0)
        {
            m_inverted = -1;
        }
    }

    @Override
    public void execute() {

        m_output = MathUtil.clamp(m_pid.calculate(m_currentAngle, DriveConstants.kBalanceSetpoint), -DriveConstants.kBalanceMaxOutput, DriveConstants.kBalanceMaxOutput);
        System.out.println(m_output);
        if(m_usePitch) {
            m_drive.driveHeading(m_output, 0, m_inverted*Math.PI/2, true);
        }
        else {
            m_drive.driveHeading(m_output, 0, 0, true);
        }
    }

    @Override
    public boolean isFinished() {
        return m_pid.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.stop();
        //TODO: Make the wheels into X to prevent rolling off
    }
}   