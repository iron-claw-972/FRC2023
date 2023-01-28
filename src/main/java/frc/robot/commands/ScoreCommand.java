package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.Robot.Teams;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.Node;

public class ScoreCommand extends SequentialCommandGroup{
  
  public ScoreCommand(){
    this(Robot.drive/*, Robot.(scoring subsystem)*/);
  }

  public ScoreCommand(Drivetrain drive/*, other subsystem*/){
    addRequirements(drive/*, other subsystem*/);
    if(Robot.selectedNode==null){
        addCommands(new DoNothing());
    }else{
      addCommands(
          new ParallelCommandGroup(
              new MoveToPose(Robot.selectedNode.scorePose, drive),
              new MoveArm(/*parameters*/)
          )
      );
    }
  }
}
