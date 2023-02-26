package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.GoToPose;
import frc.robot.subsystems.Drivetrain;

public class AutoPathNoPlan extends SequentialCommandGroup{

    private Drivetrain m_drive;
    private Pose2d m_pose, startPose;
    private Rotation2d rot;

    public AutoPathNoPlan(Drivetrain drive)  {
        m_drive = drive;

        rot = new Rotation2d(0);

        m_pose = new Pose2d(4.04, 0, rot);
        startPose = new Pose2d();

        
        addCommands(
            //deposit
            new GoToPose(m_drive, m_pose), //it goes to the place we want it to
            //intake
            new GoToPose(m_drive, startPose)// it goes back to start position

        );
    }
}
