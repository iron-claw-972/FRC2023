package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.swerve.DriveConstants;
import frc.robot.subsystems.Drivetrain;

public class BalanceCommand extends CommandBase {

    private Drivetrain m_drive;
    private final PIDController m_pid;

    private double m_currentAngle, m_output;

    private boolean m_usePitch;

    private int m_inverted;

    Timer timer = new Timer();

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
        if(m_drive.getAngleHeading() < 0)  {
            m_inverted = -1;
        }
    }

    @Override
    public void execute() {
        timer.start();
        m_currentAngle = m_drive.getPitch();
        m_output = MathUtil.clamp(m_pid.calculate(m_currentAngle), -DriveConstants.kBalanceMaxOutput, DriveConstants.kBalanceMaxOutput);
        System.out.println(m_pid.calculate(m_currentAngle));
        if(m_usePitch) {
            m_drive.driveHeading(-m_output, 0, m_inverted*Math.PI/2, true);
            System.out.println("PITCHHHH");
        }
        else {
            m_drive.driveHeading(-m_output, 0, 0, true);
            System.out.println(timer.get());
            if (timer.get() >= 0.5 && timer.get() <= 0.8)
            {
                m_drive.stop();
                timer.reset();
                System.out.println("Test");
            }

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