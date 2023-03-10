package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.swerve.DriveConstants;
import frc.robot.subsystems.Drivetrain;

public class BalanceCommand extends CommandBase {
  
  private Drivetrain m_drive;
  private final PIDController m_pid;
  
  private double m_currentAngle, m_output;
  private boolean m_usePitch;
  private boolean m_inverted;
  private boolean m_isStopping = false;
  private Timer m_timer = new Timer();
  
  public BalanceCommand(Drivetrain drive) {
    m_drive = drive;
    m_pid = drive.getBalanceController();
    m_pid.setTolerance(DriveConstants.kBalanceToleranceDegrees);
    addRequirements(drive);
  }
  
  @Override
  public void initialize() {
    m_pid.setSetpoint(0);

    //Determines whether to use roll or pitch
    if (Math.abs(m_drive.getYaw().getRadians()) < Math.PI/4) {
      m_usePitch = true;
      m_inverted = false;
    } else if (Math.abs(m_drive.getYaw().getRadians()) > 3 * Math.PI/4) {
      m_usePitch = true;
      m_inverted = true;
    } else {
      m_usePitch = false;
      m_inverted = false;
      if (m_drive.getYaw().getRadians() > 0) {
        m_inverted = true;
      }
    }

    System.out.println("BALANCING: " + (m_usePitch ? "PITCH" : "ROLL") + (m_inverted ? "INVERTED" : ""));

    m_drive.setIsBalancingOnChargeStation(true);
    m_timer.reset();
    m_isStopping = false;
  }
  
  @Override
  public void execute() {
    // starts the timer if it hasn't already been started
    m_timer.start();
    
    m_currentAngle = m_usePitch ? m_drive.getPitch().getDegrees() : m_drive.getRoll().getDegrees();

    m_output = MathUtil.clamp(m_pid.calculate(m_currentAngle), -DriveConstants.kBalanceMaxOutput, DriveConstants.kBalanceMaxOutput);

    // TODO: consider using heading PID to keep drive straight
    m_drive.drive(-m_output, 0, 0, true, true);
    
    // after DriveConstants.kBalanceNoStopPeriod, will stop periodically to give charge station time to balance. See constants.
    if (m_isStopping && m_timer.get() >= DriveConstants.kBalanceStopInterval) {
      m_drive.stop();
      if (m_timer.get() >= DriveConstants.kBalanceStopDuration + DriveConstants.kBalanceStopInterval) {
        m_timer.reset();
      }
    }

    if (m_timer.get() > DriveConstants.kBalanceNoStopPeriod) {
      m_isStopping = true;
    }
  }
  
  @Override
  public boolean isFinished() {
    return false;
  }
  
  @Override
  public void end(boolean interrupted) {
    m_drive.stop();
    m_drive.setIsBalancingOnChargeStation(false);
  }
}   