package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    
    double forwardTranslation = m_driver.getForwardTranslation();
    double sideTranslation = m_driver.getSideTranslation();
    double rotation = m_driver.getRotation();

    SmartDashboard.putNumber("Driver forward ", forwardTranslation);
    SmartDashboard.putNumber("Driver side ", sideTranslation);
    SmartDashboard.putNumber("Driver rot ", rotation);

    double slowFactor = m_driver.getIsSlowMode() ? DriveConstants.kSlowDriveFactor : 1;

    forwardTranslation *= slowFactor;
    sideTranslation *= slowFactor;
    rotation *= m_driver.getIsSlowMode() ? DriveConstants.kSlowRotFactor : 1;

    int allianceReversal = DriverStation.getAlliance() == Alliance.Blue ? 1 : -1;
    forwardTranslation *= allianceReversal;
    sideTranslation *= allianceReversal;

    if (m_driver.getIsAlign()) {
      m_swerve.driveHeading(
        forwardTranslation,
        sideTranslation,
        (Math.abs(m_swerve.getYaw().getRadians()) > Math.PI / 2) ? Math.PI : 0,
        true
      );
    } else {
      m_swerve.drive(
        forwardTranslation,
        sideTranslation,
        rotation,
        true,
        true
      );
    }
  }
}
