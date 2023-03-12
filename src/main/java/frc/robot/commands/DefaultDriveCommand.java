package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.swerve.DriveConstants;
import frc.robot.controls.BaseDriverConfig;
import frc.robot.subsystems.Drivetrain;

/**
 * Default drive command. Drives robot using driver controls.
 */
public class DefaultDriveCommand extends CommandBase {
  private final Drivetrain m_swerve;    
  private final BaseDriverConfig m_driver;

  public DefaultDriveCommand(
    Drivetrain swerve,
    BaseDriverConfig driver
  ) {
    m_swerve = swerve;
    m_driver = driver;
    addRequirements(swerve);
  }

  @Override
  public void execute() {
    
    double slowFactor = m_driver.getIsSlowMode() ? DriveConstants.kSlowDriveFactor : 1;
    double slowRotFactor = m_driver.getIsSlowMode() ? DriveConstants.kSlowRotFactor : 1;

    int reversedForRed = DriverStation.getAlliance() == Alliance.Blue ? 1 : -1;

    /* Drive */
    m_swerve.drive(
      m_driver.getForwardTranslation() * slowFactor * reversedForRed,
      m_driver.getSideTranslation() * slowFactor * -reversedForRed,
      m_driver.getRotation() * slowRotFactor,
      m_driver.getIsFieldRelative(),
      true
    );
  }
}
