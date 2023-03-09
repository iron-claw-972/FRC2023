package frc.robot.commands.vision;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.Vision;

// TODO: COMMENTS!
/**
 * 
 */
public class TestVisionAlignment extends CommandBase{
  private Drivetrain m_drive;
  private Vision m_vision;
  private double m_setpoint;
  private double m_mostRecentAngle;
  // private PIDController m_pid = new PIDController(1, 0.01, 0.1);

  public TestVisionAlignment(double targetAngle, Drivetrain drive, Vision vision){
    addRequirements(drive);
    m_setpoint = targetAngle;
    m_drive = drive;
    m_vision = vision;
    m_mostRecentAngle = m_setpoint + Math.PI;
  }

  @Override
  public void initialize() {
    m_drive.getRotationController().reset();
  }

  @Override
  public void execute() {
    m_mostRecentAngle = getAngle();
    double speed = m_drive.getRotationController().calculate(m_mostRecentAngle, m_setpoint);
    m_drive.drive(0, 0, speed, false, false);
  }

  @Override
  public void end(boolean interrupted) {
    //print vision angle
    System.out.printf("\nExact angle: %.4f degrees\n", Units.radiansToDegrees(getAngle()));
    m_drive.stop();
  }

  @Override
  public boolean isFinished(){
    return m_drive.getRotationController().atSetpoint();
  }

  private double getAngle(){
    if (m_vision.getPose2d(m_drive.getPose()) != null) {
      m_mostRecentAngle = m_vision.getPose2d(m_drive.getPose()).getRotation().getRadians();
    }
    return m_mostRecentAngle;
  }
}
