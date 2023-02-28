package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AlignWheelsToZero;
import frc.robot.commands.BalanceCommand;
import frc.robot.commands.GoToPose;
import frc.robot.subsystems.Drivetrain;

public class EngageFromCenterGrid extends SequentialCommandGroup{

    private Drivetrain m_drive;
    private Pose2d poseCharge;
    private Rotation2d startRot;

    /*
     * Drives to charge station and engages. Starts in center grid. 
     */
    public EngageFromCenterGrid(Drivetrain drive)  {
        
        m_drive = drive;

        startRot = new Rotation2d(0);
        
        poseCharge = new Pose2d(4, 0, startRot);

        addCommands(
            new AlignWheelsToZero(m_drive),
            new GoToPose(m_drive, poseCharge),
            new BalanceCommand(m_drive)

        );
    }
}
