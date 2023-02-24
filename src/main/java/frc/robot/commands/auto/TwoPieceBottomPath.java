package frc.robot.commands.auto;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.ExtendToPosition;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.FourBarArm;
import frc.robot.util.PathGroupLoader;


public class TwoPieceBottomPath extends SequentialCommandGroup{
    
    public TwoPieceBottomPath(Drivetrain m_drive, FourBarArm m_arm, double armSetPoint){
        addCommands(
            new ExtendToPosition(m_arm, armSetPoint),
            
            // ** deposit already loaded cone in the high node of the bottom grid.  

            new PathPlannerCommand(PathGroupLoader.getPathGroup("To Center And Back"), 0, m_drive, true), 
        
            // ** intake a cone 
            
            new PathPlannerCommand(PathGroupLoader.getPathGroup("To Center And Back"), 1, m_drive, false)

            // ** Deposit cone high nodes in the same grid and stop in original position

        );
    }
};
  
  


