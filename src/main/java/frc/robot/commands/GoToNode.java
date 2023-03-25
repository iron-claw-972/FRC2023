package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.PathPoint;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.auto.PathPlannerCommand;
import frc.robot.controls.Operator;
import frc.robot.subsystems.Drivetrain;

public class GoToNode extends CommandBase {

  private Operator m_operator;
  private Drivetrain m_drive;
  private PathPlannerCommand m_command;

  /**
   * Uses PathPlanner to go to the selected node
   * @param operator The operator
   * @param drive The drivetrain
   */
  public GoToNode(Operator operator, Drivetrain drive) {
    m_operator = operator;
    m_drive = drive;
  }

  /**
   * Creates the PathPlanner command and schedules it
   */
  @Override
  public void initialize() {
    // Gets the current position of the robot for the start of the path
    PathPoint point1 = PathPoint.fromCurrentHolonomicState(
      m_drive.getPose(),
      m_drive.getChassisSpeeds()
    );
    // Uses the operator's selected node to find the end point for the path
    PathPoint point2 = new PathPoint(
      m_operator.getSelectedNode().scorePose.getTranslation(),
      m_operator.getSelectedNode().scorePose.getRotation(),
      m_operator.getSelectedNode().scorePose.getRotation(),
      0
    );
    // Creates the command using the two points
    m_command = new PathPlannerCommand(
      new ArrayList<PathPoint>(List.of(point1, point2)),
      m_drive,
      false
    );
    // Starts the command
    m_command.schedule();
  }

  /**
   * Stops the command and the drivetrain
   * @param interrupted If the command is interrupted
   */
  @Override
  public void end(boolean interrupted) {
    m_command.cancel();
    m_drive.stop();
  }

  /**
   * Returns if the PathPlannerCommand exists and is finished
   * @return If the GoToNode command is finished
   */
  @Override
  public boolean isFinished() {
    return m_command != null && m_command.isFinished();
  }
}
