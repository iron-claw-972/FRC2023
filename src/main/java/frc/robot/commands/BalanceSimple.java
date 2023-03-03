package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class BalanceSimple extends CommandBase {
  private final Drivetrain m_drive;

  private double balanceEffort;
  private double turningEffort;

  private final PIDController m_balancePID = new PIDController(0.05, 0, 0.006);

  public BalanceSimple(Drivetrain drive) {
    m_drive = drive;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    m_balancePID.setSetpoint(0);
    m_drive.setIsBalancingOnChargeStation(true);
  }

  @Override
  public void execute() {
    turningEffort = 0;
    balanceEffort = m_balancePID.calculate(m_drive.getPitch().getDegrees());
    m_drive.drive(balanceEffort, 0, turningEffort, false, true);
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.stop();
    m_drive.setIsBalancingOnChargeStation(false);
  }

  @Override
  public boolean isFinished() {
    return DriverStation.isAutonomousEnabled() ? false : Math.abs(m_drive.getPitch().getDegrees()) < 2;
  }
}
