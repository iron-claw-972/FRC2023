package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class MoveToPose extends CommandBase{
  
  /**
   * A command that currently does nothing
   * @param pose The pose to move to
   * @param drive The drivetrain
   */
  public MoveToPose(Pose2d pose, Drivetrain drive){
    addRequirements(drive);
  }
}
