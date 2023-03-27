package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.auto.PathPlannerCommand;
import frc.robot.subsystems.Drivetrain;

public class GoToPose extends CommandBase {

  private Drivetrain m_drive;
  private CommandBase m_command;
  private Supplier<Pose2d> m_poseSupplier;

  /**
   * Uses PathPlanner to go to a pose
   * @param poseSupplier The supplier for the pose to use
   * @param drive The drivetrain
   */
  public GoToPose(Supplier<Pose2d> poseSupplier, Drivetrain drive) {
    m_poseSupplier = poseSupplier;
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
    
      // set the control lengths. This controls how strong the heading is
      // aka how much the robot will curve to get to the point. 
      // We want it to follow a straight line, and with swerve, it isn't too necessary.
    ).withControlLengths(0.001, 0.001);

    // get the desired score pose
    Pose2d pose = m_poseSupplier.get();

    // Uses the pose to find the end point for the path
    PathPoint point2 = new PathPoint(
      pose.getTranslation(),
      pose.getRotation(),
      pose.getRotation(),
      0
      // set the control lengths. This controls how strong the heading is
      // aka how much the robot will curve to get to the point. 
      // We want it to follow a straight line, and with swerve, it isn't too necessary.
    ).withControlLengths(0.001, 0.001);

    // Creates the command using the two points
    m_command = new PathPlannerCommand(
      new ArrayList<PathPoint>(List.of(point1, point2)),
      m_drive,
      false
    );

    // get the distance to the pose.
    double dist = m_drive.getPose().minus(pose).getTranslation().getNorm();

    // if greater than 4m or less than 20 cm, don't run it. If the path is too small pathplanner makes weird paths.
    if (dist > 4) {
      m_command = new DoNothing();
      DriverStation.reportWarning("Alignment Path too long, doing nothing, GoToNode.java", false);
    } else if (dist < 0.2) {
      m_command = new DoNothing();
      DriverStation.reportWarning("Alignment Path too short, doing nothing, GoToNode.java", false);
    } else {
      // Creates the command using the two points
      m_command = new PathPlannerCommand(
        new ArrayList<PathPoint>(List.of(point1, point2)),
        m_drive,
        false
      );
    }

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
