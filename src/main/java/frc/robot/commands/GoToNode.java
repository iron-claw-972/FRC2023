package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.PathPoint;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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

  @Override
  public void initialize(){
    // Gets the current position of the robot for the start of the path
    PathPoint point1 = PathPoint.fromCurrentHolonomicState(
      m_drive.getPose(),
      m_drive.getChassisSpeeds()
    );
    // Uses the operator's selected node to find the end point for the path
    PathPoint point2 = new PathPoint(
      m_operator.getSelectedNode().scorePose.getTranslation(),
      m_operator.getSelectedNode().scorePose.getRotation(),
      m_operator.getSelectedNode().scorePose.getRotation()
    );
    m_command = new PathPlannerCommand(
    new ArrayList<PathPoint>(List.of(point1, point2)), m_drive, false);
    m_command.schedule();
  }

  @Override
  public void end(boolean interrupted){
    m_drive.stop();
    m_command.cancel();
  }

  @Override
  public boolean isFinished(){
    return m_command!=null && m_command.isFinished();
  }
}
