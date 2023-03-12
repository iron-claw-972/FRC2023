
package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.BalanceCommand;
import frc.robot.commands.GoToPose;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.PathGroupLoader;

public class EngageFromGrid1 extends SequentialCommandGroup{

    private Drivetrain m_drive;

    public EngageFromGrid1(Drivetrain drive)  {

        addCommands(
            new PathPlannerCommand(PathGroupLoader.getPathGroup("Engage From Grid 1"), 0 , m_drive, true), //intake
            new BalanceCommand(m_drive)
            );
        }
    }
    