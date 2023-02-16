package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.arm.ExtendToPosition;
import frc.robot.constants.ArmConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.FourBarArm;

// Basic command structure for how to use selectedNode
public class ScoreCommand extends SequentialCommandGroup{
  
  /**
   * A sequential command group that will move to the selected pose and score
   * MoveToPose can be replaced with whatever other command does that later
   * ExtendToPositon should not change here
   * The rest of the commands (move elevater up, let go of piece, move elevater back) still needs to be done
   * @param drive The drivetrain
   */
  public ScoreCommand(Drivetrain drive, FourBarArm arm){
    addRequirements(drive/*, other subsystem*/);
    if(RobotContainer.selectedNode==null){
        addCommands(new DoNothing());
    }else{
      addCommands(
          new ParallelCommandGroup(
              new MoveToPose(RobotContainer.selectedNode.scorePose, drive),
              new ExtendToPosition(arm, RobotContainer.selectedNode.row==1?ArmConstants.klowPosition:RobotContainer.selectedNode.row==2?ArmConstants.kmiddlePosition:ArmConstants.ktopPosition)
          ),
          // Let go of game piece
          new ParallelCommandGroup(
            // Add move elevater here
            new ExtendToPosition(arm, ArmConstants.kinitialPosition)
          )
      );
    }
  }
}
