package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.PathPoint;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auto.PathPlannerCommand;
import frc.robot.controls.Operator;
import frc.robot.subsystems.Drivetrain;

public class GoToNode extends SequentialCommandGroup {
  /**
   * Goes to the selected node using PathPlanner
   * Creates a new path, uses it, and then stops the robot
   * @param operator The operator
   * @param drive The drivetrain
   */
  public GoToNode(Operator operator, Drivetrain drive) {
    PathPoint point1 = new PathPoint(
      drive.getPose().getTranslation(), 
      drive.getPose().getRotation(), 
      drive.getFieldRelativeHeading()
    );
    PathPoint point2 = new PathPoint(
      operator.getSelectedNode().scorePose.getTranslation(),
      operator.getSelectedNode().scorePose.getRotation(),
      operator.getSelectedNode().scorePose.getRotation()
    );

    addRequirements(drive);
    addCommands(new PathPlannerCommand(
      new ArrayList<PathPoint>(List.of(point1, point2)), drive, false),
      new InstantCommand(()->drive.stop())
    );
  }
}
