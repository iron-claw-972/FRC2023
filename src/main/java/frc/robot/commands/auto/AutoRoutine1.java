package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.GoToPose;
import frc.robot.commands.arm.ExtendToPosition;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.FourBarArm;
import frc.robot.util.PathGroupLoader;

public class AutoRoutine1 extends SequentialCommandGroup{
    /*move forward slightly, 
    piece goes up elevator
    elevator deposit,
    drive to center game piece following pathplanner commands
    intake
    drive to charge station
    engage
    */
     
    private Drivetrain m_drive;
    private FourBarArm m_arm;

    private double armSetpoint;

    public AutoRoutine1(Drivetrain drive, GoToPose goTo, FourBarArm arm)  {
        m_drive = drive;
        m_arm = arm;
        addCommands(
            new ExtendToPosition(m_arm, armSetpoint),
            //deposit
            new PathPlannerCommand(PathGroupLoader.getPathGroup("BottomSimpleLine1"), 0 ,m_drive, true)
            //intake
        );
    }
}

