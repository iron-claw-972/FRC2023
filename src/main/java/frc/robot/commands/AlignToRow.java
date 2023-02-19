package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.PathPoint;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.controls.Operator;
import frc.robot.subsystems.Drivetrain;

/**
 * Go to 1-9 row in grid area
 */
public class AlignToRow extends SequentialCommandGroup {

  private final Drivetrain m_drive;
  
  public AlignToRow(Drivetrain drive) {
    m_drive = drive;
    addRequirements(drive);
    addCommands(
      new PathPlannerCommand(new ArrayList<PathPoint>(List.of(
        PathPoint.fromCurrentHolonomicState(m_drive.getPose(), m_drive.getChassisSpeeds()),
        new PathPoint(Operator.selectedNode.scorePose.getTranslation(), Operator.selectedNode.scorePose.getRotation())
      )), m_drive)
    );
  }
}
