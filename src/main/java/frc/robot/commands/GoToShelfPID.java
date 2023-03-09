package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.Drivetrain;

/**
 * Uses PID to make the robot go to the shelf
 */
public class GoToShelfPID extends CommandBase {

  private final Drivetrain m_drive;
  public GoToShelfPID(Drivetrain drive) {
    m_drive = drive;    
    addRequirements(drive);
  }

  @Override
  public void initialize(){
    m_drive.setAllOptimize(true);
    m_drive.getXController().reset();
    m_drive.getYController().reset();
    m_drive.getRotationController().reset();
  }
  
  @Override
  public void execute() {
    double x = DriverStation.getAlliance()==Alliance.Blue?FieldConstants.kBlueShelfX:FieldConstants.kRedShelfX;
    double y = FieldConstants.kShelfY;
    double rot = DriverStation.getAlliance()==Alliance.Blue?0:Math.PI;
    m_drive.runChassisPID(x, y, rot);
  }
  
  @Override
  public void end(boolean interrupted) {
    m_drive.stop();
  }

  @Override
  public boolean isFinished(){
    double x = DriverStation.getAlliance()==Alliance.Blue?FieldConstants.kBlueShelfX:FieldConstants.kRedShelfX;
    double y = FieldConstants.kShelfY;
    double rot = DriverStation.getAlliance()==Alliance.Blue?0:Math.PI;
    return new Translation2d(x, y).getDistance(
      m_drive.getPose().getTranslation()) < 0.01 &&
      Math.abs(Math.abs(m_drive.getPose().getRotation().getRadians())-
      rot) < Units.degreesToRadians(1);
  }
}
