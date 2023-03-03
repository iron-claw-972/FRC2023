package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.swerve.DriveConstants;
import frc.robot.subsystems.Drivetrain;

public class BalanceCommand extends CommandBase {
  
  private Drivetrain m_drive;
  private final PIDController m_pid = new PIDController(
    DriveConstants.kBalanceP, 
    DriveConstants.kBalanceI, 
    DriveConstants.kBalanceD
  );
  
  private double m_currentAngle, m_output;
  private boolean m_usePitch;
  private boolean m_inverted;
  private Timer m_timer = new Timer();
  
  public BalanceCommand(Drivetrain drive) {
    m_drive = drive;
    m_pid.setTolerance(DriveConstants.kBalanceTolerance);
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

    m_drive.setIsBalancingOnChargeStation(true);
  }
  
  @Override
  public void execute() {
    // starts the timer if it hasn't already been started
    m_timer.start();
    
    m_currentAngle = m_usePitch ? m_drive.getPitch().getDegrees() : m_drive.getRoll().getDegrees();

    m_output = MathUtil.clamp(m_pid.calculate(m_currentAngle), -DriveConstants.kBalanceMaxOutput, DriveConstants.kBalanceMaxOutput);

    if (m_usePitch) {
      m_drive.driveHeading(-m_output, 0, (m_inverted ? 0 : Math.PI), true);
    } else {
      m_drive.driveHeading(-m_output, 0, (m_inverted ? -Math.PI/2 : Math.PI/2), true); // TODO: inversion may be incorrect
    }
    
    if (m_timer.get() >= DriveConstants.kBalanceStartTime && m_timer.get() <= DriveConstants.kBalanceEndTime) {
      m_drive.stop();
      m_timer.reset();
    }
  }
  
  @Override
  public boolean isFinished() {
    return m_pid.atSetpoint();
  }
  
  @Override
  public void end(boolean interrupted) {
    m_drive.stop();
    m_drive.setIsBalancingOnChargeStation(false);
  }
}   