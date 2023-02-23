package frc.robot.commands.auto;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.FourBarArm;


public class IntakeBottomPieceCommand extends SequentialCommandGroup{
    private Drivetrain m_drive;
    private FourBarArm m_Arm;
    
    public IntakeBottomPieceCommand(){
    
        addCommands(
        //deposit preloaded cone into high node of bottom grid

        // **path not created yet, but have it drive to center

        // intake cone game piece

        // ** path not created yet, but have it drive back
        
        // deposit cone high nodes in same grid
        );
            
        
    }
};
  
  


