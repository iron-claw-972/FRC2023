package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.controls.Operator;
import frc.robot.subsystems.Drivetrain;

/**
 * Uses PID to make the robot go to the selected pose
 */
public class MoveToSelectedPose extends CommandBase {

  private final Drivetrain m_drive;
  
  public MoveToSelectedPose(Drivetrain drive) {
    m_drive = drive;    
    addRequirements(drive);
  }
  
  @Override
  public void execute() {
    m_drive.setAllOptimize(true);
    
    double x = Operator.selectedNode.scorePose.getX();
    double y = Operator.selectedNode.scorePose.getY();
    double rot = Operator.selectedNode.scorePose.getRotation().getRadians();

    System.out.printf("Moving to (%.2f, %.2f) at %.2f degrees", x, y, Units.radiansToDegrees(rot));

    m_drive.runChassisPID(x, y, rot);
  }
  
  @Override
  public void end(boolean interrupted) {
    m_drive.stop();
  }
}
