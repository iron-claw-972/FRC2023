package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.Teams;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.Node;

public class ScoreCommand extends SequentialCommandGroup{
  

  public ScoreCommand(Drivetrain drive/*, other subsystem*/){
    addRequirements(drive/*, other subsystem*/);
    if(RobotContainer.selectedNode==null){
        addCommands(new DoNothing());
    }else{
      addCommands(
          new ParallelCommandGroup(
              new MoveToPose(RobotContainer.selectedNode.scorePose, drive),
              new MoveArm(drive/*other parameters*/)
          )
      );
    }
  }
}
