package frc.robot.commands.drive;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.PathPlannerCommand;
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
        // TODO
        new PathPoint(new Translation2d(), new Rotation2d(), new Rotation2d())
      )), m_drive)
    );
  }
}
