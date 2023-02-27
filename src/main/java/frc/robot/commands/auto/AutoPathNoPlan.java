package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.GoToPose;
import frc.robot.subsystems.Drivetrain;

public class AutoPathNoPlan extends SequentialCommandGroup{

    private Drivetrain m_drive;
    private Pose2d centerPose, initialPose;
    private Rotation2d rot, startRot;

    public AutoPathNoPlan(Drivetrain drive)  {
        //DRIVES TO CENTER LINE, THEN DRIVES BACK. CAN BE DONE ON EITHER SIDE OF CHARGE STATION
        m_drive = drive;

        startRot = new Rotation2d(0);
        rot = new Rotation2d(Math.PI);

        centerPose = new Pose2d(5.5, 0, rot);
        initialPose = new Pose2d(-.5,0, startRot);//it is -0.5 because for some reason when it is 0 it doesnt come back all the way

        addCommands(
            new GoToPose(m_drive, centerPose), 
            new WaitCommand(1),
            new GoToPose(m_drive, initialPose)
           
        );
    }
}
