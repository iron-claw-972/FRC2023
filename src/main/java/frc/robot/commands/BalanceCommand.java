package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.Functions;

public class BalanceCommand extends CommandBase {

    private Drivetrain m_drive;
    private final PIDController m_pid;

    private double m_currentAngle, m_output;

    private boolean m_balanceMode;
 
    public BalanceCommand(Drivetrain drive) {
        m_drive = drive;
        addRequirements(drive);
        m_pid = new PIDController(DriveConstants.kBalanceP, DriveConstants.kBalanceI, DriveConstants.kBalanceD);
        m_pid.setTolerance(DriveConstants.kBalanceTolerance);
    }

    /*
     * Determines the values of the vectors of pitch, yaw, and roll. 
     */
    @Override
    public void initialize() {
        m_pid.setSetpoint(0);
        if(90 - m_drive.getAngleHeading() > m_drive.getAngleHeading())  { //Determines whether to use roll or pitch
            m_balanceMode = false;
        }
        else  {
            m_balanceMode = true;
        }
    }

    @Override
    public void execute() {
        m_currentAngle = Functions.calculateHypotenuse(m_drive.getAngleHeading(), Functions.calculateHypotenuse(m_drive.getPigeon().getPitch(), m_drive.getPigeon().getRoll()));
        m_output = MathUtil.clamp(m_pid.calculate(m_currentAngle, DriveConstants.kBalanceSetpoint), -DriveConstants.kBalanceMaxOutput, DriveConstants.kBalanceMaxOutput);
        if(m_balanceMode) {
            m_drive.driveHeading(m_output, 0, 0, true);
        }
        else {
            m_drive.driveHeading(m_output, 0, 90, true);
        }
    }

    @Override
    public boolean isFinished() {
        return m_pid.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.stop();
    }
}   