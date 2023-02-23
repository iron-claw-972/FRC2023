package frc.robot.commands.auto;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.FourBarArm;
import frc.robot.util.PathGroupLoader;


public class IntakeBottomPieceCommand extends SequentialCommandGroup{
    private Drivetrain m_drive;
    private FourBarArm m_Arm;
    
    public IntakeBottomPieceCommand(Drivetrain m_drive, FourBarArm m_Arm){
        addCommands(
        // ** deposit already loaded cone in the high node of the bottom grid.  

       // first part of command makes it drive to center 
        new PathPlannerCommand(PathGroupLoader.getPathGroup("To Center And Back"), 0, m_drive, true), 
       
        // ** intake a cone 

        // second part of command makes it drive back
        new PathPlannerCommand(PathGroupLoader.getPathGroup("To Center And Back"), 1, m_drive, true)

        // ** Deposit cone high nodes in the same grid and stop in original position

        );
    }
};
  
  


