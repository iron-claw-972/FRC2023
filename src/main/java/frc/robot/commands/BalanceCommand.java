package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class BalanceCommand extends CommandBase {

    Drivetrain m_drive = new Drivetrain(null, null);

    private double kAP, kAI, kAD, startAngle, endAngle

    PIDController m_pid = new PIDController(kAP, kAI, kAD);


    public void initialize()
    {
        startAngle = m_drive.getAngleHeading();
        endAngle = 0;
    }

    public void execute()
    {
        m_drive.setPigeonYaw(m_pid.calculate(startAngle, endAngle));
    }

    public boolean isFinished()
    {
        if(m_drive.getAngleHeading() + 0.05 >= 0 && m_drive.getAngleHeading() - 0.05 <= 0)
        {
            return true;
        }
        return false;
    }

    public void end()
    {
        m_drive.stop();
    }
}
