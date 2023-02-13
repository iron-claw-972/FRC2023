package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.Node;

// Basic command structure for how to use selectedNode
public class ScoreCommand extends SequentialCommandGroup{
  
  /**
   * A sequential command group that will move to the selected pose and score
   * The commands it calls are unfinished
   * @param drive The drivetrain
   */
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
