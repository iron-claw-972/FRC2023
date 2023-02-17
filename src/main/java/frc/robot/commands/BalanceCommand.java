package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.Functions;

public class BalanceCommand extends CommandBase {

    private Drivetrain m_drive;

    private double startAngle, endPoint;

    private boolean mode;
 
    public BalanceCommand(Drivetrain drive) {
        m_drive = drive;
    }

    /*
     * Determines the values of the vectors of pitch, yaw, and roll. 
     */
    public void initialize()  {

        endPoint = 0;
        startAngle = Functions.calculateHypotenuse(m_drive.getAngleHeading(), Functions.calculateHypotenuse(m_drive.getPigeon().getPitch(), m_drive.getPigeon().getRoll()));

        if(90 - m_drive.getAngleHeading() > m_drive.getAngleHeading())  { //Determines whether to use roll or pitch
            mode = false;
        }

        else  {
            mode = true;
        }
    }

    /**
     * Makes the robot wiggle around PID like until charge station is balanced. 
     * ySpeed is zero for now, will add yaw at later pull request.
     *  
     */
    public void execute()  {
    
      m_drive.m_balanceController.setTolerance(0.1);

      if(mode)   {
        m_drive.driveHeading(m_drive.m_balanceController.calculate(endPoint, startAngle), 0, 0, true);
      }
      else   {
        m_drive.driveHeading(m_drive.m_balanceController.calculate(endPoint, startAngle), 0, 90, true);
      }
    }

    /**
    * Checks to see if robot is within acceptable range to determine if it is balanced. 
    * @return True if the robot is within range, false if it is not.
    */
    public boolean isFinished() {
        return m_drive.m_balanceController.atSetpoint();
    }

    public void end()  {
        m_drive.stop();
    }
}