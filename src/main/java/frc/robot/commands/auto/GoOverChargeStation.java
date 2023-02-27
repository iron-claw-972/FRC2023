package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.BalanceCommand;
import frc.robot.commands.GoToPose;
import frc.robot.subsystems.Drivetrain;

public class GoOverChargeStation extends SequentialCommandGroup{

    private Drivetrain m_drive;
    private Pose2d initialPose, chargePose, gamePiecePose, rotatedPose;
    private Rotation2d startRot, endRot;

    public GoOverChargeStation(Drivetrain drive)  {
        m_drive = drive;

        startRot = new Rotation2d(0);
        endRot = new Rotation2d(0);
        
        initialPose = new Pose2d(0, 0, endRot);
        chargePose = new Pose2d(4, 0, endRot);
        rotatedPose = new Pose2d(0, 0, endRot);
        gamePiecePose = new Pose2d(7.5, 0, endRot);
        
        addCommands(
            new GoToPose(m_drive, rotatedPose),
            new GoToPose(m_drive, gamePiecePose),
            new GoToPose(m_drive, chargePose),
            new BalanceCommand(drive)


        );
    }
}
