package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.controls.BaseDriverConfig;
import frc.robot.controls.GameControllerDriverConfig;
import frc.robot.subsystems.Drivetrain;

/**
 * Default drive command. Drives robot using driver controls.
 */
public class DefaultDriveCommand extends CommandBase {

  private final Drivetrain m_drive;
  private final GameControllerDriverConfig m_driver;
  
  public DefaultDriveCommand(Drivetrain drive, GameControllerDriverConfig driver) {
    m_drive = drive;
    m_driver = driver;
    
    addRequirements(drive);
  }
  
  @Override
  public void execute() {
    m_drive.setAllOptimize(true);
    
    m_driver.updateSettings();
    double xSpeed = m_driver.getForwardTranslation();
    double ySpeed = m_driver.getSideTranslation();
    double rot = m_driver.getRotation();
    double slowFactor = m_driver.isSlowSpeed().getAsBoolean() ? 0.5 : 1;

    m_drive.drive(xSpeed * slowFactor, ySpeed * slowFactor, rot, true);//m_driver.isSlowSpeed().getAsBoolean()
  }
  
  @Override
  public void end(boolean interrupted) {
    m_drive.drive(0.0, 0.0, 0.0, false);
  }
}
