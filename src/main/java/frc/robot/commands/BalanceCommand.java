package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.Functions;

public class BalanceCommand extends CommandBase {

    private Drivetrain m_drive;

    public BalanceCommand(Drivetrain drive) {
        m_drive = drive;
    }

    private double kBalanceP, kBalanceI, kBalanceD, startAngle, endAngle;

    


    public void initialize()  {
        endAngle = 0;
        startAngle = Functions.calculateHypotenuse(m_drive.getAngleHeading(), Functions.calculateHypotenuse(m_drive.getPigeon().getPitch(), m_drive.getPigeon().getRoll()));
    }

    public void execute()
    {
      m_drive.m_balanceController.calculate(startAngle, endAngle);
      //ySpeed and heading are zero for now, will add yaw at later pull request. 
      m_drive.driveHeading(m_drive.m_balanceController.calculate(startAngle, endAngle), 0, 0, true);

      m_drive.m_balanceController.setTolerance(0.1);
    }

    public boolean isFinished() 
    {
        if(m_drive.getAngleHeading() + 0.05 >= 0 && m_drive.getAngleHeading() - 0.05 <= 0)
        {
            return true;
        }
        ///Rotation3d rot = new Rotation3d(roll, pitch, yaw);
        return false;
    }

    public void end()    {
        m_drive.stop();
    }
}
