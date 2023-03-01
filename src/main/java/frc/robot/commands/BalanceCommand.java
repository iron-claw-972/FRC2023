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
  
  Timer m_timer = new Timer();
  
  public BalanceCommand(Drivetrain drive) {
    m_drive = drive;
    m_pid.setTolerance(DriveConstants.kBalanceTolerance);
    addRequirements(drive);
  }
  
  @Override
  public void initialize() {
    
    m_inverted = false;
    
    m_pid.setSetpoint(0);
    if(Math.abs(Math.PI/2 - Math.abs(m_drive.getAngleHeading())) > Math.PI/4)  { //Determines whether to use roll or pitch
      m_usePitch = false;
    }
    else  {
      m_usePitch = true;
    }
    if(m_drive.getAngleHeading() < Math.PI/4 || m_drive.getAngleHeading() < -Math.PI*3/4)  {
      m_inverted = true;
    }
  }
  
  @Override
  public void execute() {
    m_timer.start();
    
    m_output = MathUtil.clamp(m_pid.calculate(m_currentAngle), -DriveConstants.kBalanceMaxOutput, DriveConstants.kBalanceMaxOutput);
    
    if(m_usePitch) {
    m_currentAngle = m_drive.getPitch();
      m_drive.driveHeading(-m_output, 0, (m_inverted? -Math.PI/2: Math.PI/2), true);
    }
    else {
    m_currentAngle = m_drive.getRoll();
      m_drive.driveHeading(-m_output, 0, (m_inverted? 0: Math.PI), true); // TODO: inversion may be incorrect
    }
    
    if (m_timer.get() >= 0 && m_timer.get() <= DriveConstants.kBalanceTime) {
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
  }
}   