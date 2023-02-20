package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.subsystems.Drivetrain;

/**
 * Default drive command. Drives robot using driver controls.
 */
public class DefaultDriveCommand extends CommandBase {

  private final Drivetrain m_drive;
  
  public DefaultDriveCommand(Drivetrain drive) {
    m_drive = drive;
    
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    m_drive.setAllOptimize(true);
  }
  
  @Override
  public void execute() {
    OI.updateShuffleboard();
    double xSpeed = OI.getForwardTranslation();
    double ySpeed = OI.getSideTranslation();
    double rot = OI.getRotation();

    m_drive.drive(xSpeed, ySpeed, rot, true);
  }
  
  @Override
  public void end(boolean interrupted) {
    m_drive.drive(0.0, 0.0, 0.0, false);
  }
}
