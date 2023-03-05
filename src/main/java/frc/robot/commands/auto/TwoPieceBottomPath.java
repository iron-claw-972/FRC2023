package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.FourBarArm;
import frc.robot.util.PathGroupLoader;

public class TwoPieceBottomPath extends SequentialCommandGroup{
  
  /**
   * Deposit two Pieces doesn't cross the charge station 
   * @param m_drive drivetrain used
   * @param m_arm four bar arm used
   */
  public TwoPieceBottomPath(Drivetrain m_drive, FourBarArm m_arm){
    addCommands(
      //deposit preloaded cone in high node
      new PathPlannerCommand(PathGroupLoader.getPathGroup("To Center And Back"), 0, m_drive, true, true),
      //intake a cone 
      new PathPlannerCommand(PathGroupLoader.getPathGroup("To Center And Back"), 1, m_drive, false, true)
      //deposit cone in high node
    );
  }
}