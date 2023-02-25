package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.test.GoToPoseTest;
import frc.robot.commands.arm.ExtendToPosition;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.FourBarArm;
import frc.robot.util.PathGroupLoader;

public class EngageBottomPath extends SequentialCommandGroup{
   
    private Drivetrain m_drive;
    private FourBarArm m_arm;

    private double armSetpoint = Math.PI/6;

    public EngageBottomPath(Drivetrain drive)  {
        m_drive = drive;
        

        addCommands(
            //new ExtendToPosition(m_arm, armSetpoint), //deposit
            new WaitCommand(2),
            new PathPlannerCommand(PathGroupLoader.getPathGroup("Bottom Simple Line1"), 0 , m_drive, true), //intake
            new WaitCommand(3),
            new PathPlannerCommand(PathGroupLoader.getPathGroup("Bottom Simple Line2"), 0 , m_drive, false) //engage
    
        );
    }
}

