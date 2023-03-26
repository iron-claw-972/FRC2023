package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auto.PathPlannerCommand;
import frc.robot.controls.Operator;
import frc.robot.subsystems.Drivetrain;

public class GoToNode extends SequentialCommandGroup {

  private Operator m_operator;
  private Drivetrain m_drive;
  private DoubleSupplier m_yOffsetSup;
  private SequentialCommandGroup m_command;

  /**
   * Uses PathPlanner to go to the selected node
   * @param operator The operator
   * @param drive The drivetrain
   */
  public GoToNode(Operator operator, Drivetrain drive, DoubleSupplier yOffsetSup) {
    m_operator = operator;
    m_drive = drive;
    m_yOffsetSup = yOffsetSup;
    addRequirements(drive);
    m_command = new SequentialCommandGroup();
    addCommands(
      new InstantCommand(()->init()),
      m_command
    );
  }

  /**
   * Creates the PathPlanner command and schedules it
   */
  public void init() {
    // Gets the current position of the robot for the start of the path
    PathPoint point1 = PathPoint.fromCurrentHolonomicState(
      m_drive.getPose(),
      m_drive.getChassisSpeeds()
    ).withControlLengths(0.001, 0.001);

    // get the desired score pose
    Pose2d scorePose = m_operator.getSelectedNode().scorePose;

    // get a y offset from the supplier (currently we use the intake to offset scoring)
    double yOffset = m_yOffsetSup.getAsDouble();

    // modify pose by offsets
    scorePose.plus(new Transform2d(
      new Translation2d(0, -0.07 + yOffset), 
      new Rotation2d(0)
    ));

    // Uses the operator's selected node to find the end point for the path
    PathPoint point2 = new PathPoint(
      scorePose.getTranslation(),
      scorePose.getRotation(),
      scorePose.getRotation(),
      0
    ).withControlLengths(0.001, 0.001);

    // Creates the command using the two points
    CommandBase command = new PathPlannerCommand(
      new ArrayList<PathPoint>(List.of(point1, point2)),
      m_drive,
      false
    );

    double dist = m_drive.getPose().minus(scorePose).getTranslation().getNorm();
    if (dist > 3) {
      command = new DoNothing();
      DriverStation.reportWarning("Alignment Path too long, doing nothing, GoToNode.java", false);
    }
    if (dist < 0.2) {
      command = new DoNothing();
      DriverStation.reportWarning("Alignment Path too short, doing nothing, GoToNode.java", false);
    }

    // Adds the command
    m_command.addCommands(command);
  }

  /**
   * Stops the command and the drivetrain
   * @param interrupted If the command is interrupted
   */
  // @Override
  // public void end(boolean interrupted) {
  //   m_command.cancel();
  //   m_drive.stop();
  // }

  /**
   * Returns if the PathPlannerCommand exists and is finished
   * @return If the GoToNode command is finished
   */
  // @Override
  // public boolean isFinished() {
  //   return m_command != null && m_command.isFinished();
  // }
}
