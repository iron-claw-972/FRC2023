
package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.BalanceCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.PathGroupLoader;

public class EngageFromGrid9 extends SequentialCommandGroup{
    private Drivetrain m_drive;

    public EngageFromGrid9(Drivetrain drive)  {
        //DRIVES OUT OF COMMUNITY, THEN MOVES TO RIGHT AND BACK TO ENGAGE. THIS PATH IS FOR THE GRID POSITION CLOSEST TO BARRIER
        m_drive = drive;

        addCommands(
            new PathPlannerCommand(PathGroupLoader.getPathGroup("Engage From Grid 9"), 0 , m_drive, true), //intake
            new BalanceCommand(m_drive)
            );
        }
    }
    