package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
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
  private final PIDController m_pid;

  public DefaultDriveCommand(
    Drivetrain swerve,
    BaseDriverConfig driver
  ) {
    m_swerve = swerve;
    m_driver = driver;
    addRequirements(swerve);

    m_pid = new PIDController(0.08, 0, 0);
    m_pid.enableContinuousInput(-180, 180);
    m_pid.setTolerance(0.25, 0.25);
  }

  @Override
  public void execute() {
    
    double slowFactor = m_driver.getIsSlowMode() ? DriveConstants.kSlowDriveFactor : 1;
    double slowRotFactor = m_driver.getIsSlowMode() ? DriveConstants.kSlowRotFactor : 1;

    int reversedForRed = DriverStation.getAlliance() == Alliance.Blue ? -1 : 1;

    Rotation2d currentYaw = m_swerve.getYaw();
    double turnEffort = m_driver.getIsAlign()
        ? (m_pid.calculate(currentYaw.getDegrees(),
            ((Math.abs(currentYaw.getDegrees()) % 360 > 90 && Math.abs(currentYaw.getDegrees()) % 360 < 270) ? 180 : 0)))
        : (m_driver.getRotation() * slowRotFactor);

    /* Drive */
    m_swerve.drive(
      m_driver.getForwardTranslation() * slowFactor * reversedForRed,
      m_driver.getSideTranslation() * slowFactor * reversedForRed,
      turnEffort,
      m_driver.getIsFieldRelative(),
      true
    );
  }
}
