package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;

public class MoveToPose extends CommandBase{
  
  public MoveToPose(Pose2d pose){
    this(pose, Robot.drive);
  }

  public MoveToPose(Pose2d pose, Drivetrain drive){
    addRequirements(drive);
  }
}
