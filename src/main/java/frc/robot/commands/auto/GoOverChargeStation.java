package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.BalanceCommand;
import frc.robot.commands.GoToPose;
import frc.robot.subsystems.Drivetrain;

public class GoOverChargeStation extends SequentialCommandGroup{

    private Drivetrain m_drive;
    private Pose2d chargePose, gamePiecePose;
    private Rotation2d startRot;

    public GoOverChargeStation(Drivetrain drive)  {
        m_drive = drive;

        startRot = new Rotation2d(0);
        
        chargePose = new Pose2d(4, 0, startRot);
        gamePiecePose = new Pose2d(6.5, 0, startRot);
        
        addCommands(
            new GoToPose(m_drive, gamePiecePose),
            new GoToPose(m_drive, chargePose),
            new BalanceCommand(drive)


        );
    }
}
