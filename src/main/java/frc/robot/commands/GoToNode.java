package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.controls.Operator;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.Node;

public class GoToNode extends CommandBase {
  
  private Drivetrain m_drive;

  public GoToNode(Node node, Drivetrain drive) {
    m_drive = drive;
  }
}
