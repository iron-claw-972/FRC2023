package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.BalanceCommand;
import frc.robot.commands.GoToPose;
import frc.robot.subsystems.Drivetrain;

public class EngageNoPathplanner extends SequentialCommandGroup{

    private Drivetrain m_drive;
    private Pose2d midPose, startPose, endPose, initialPose, poseRight, poseCharge, endPoseNoRotate;
    private Rotation2d rot, startRot;

    public EngageNoPathplanner(Drivetrain drive)  {
        m_drive = drive;

        startRot = new Rotation2d(0);
        rot = new Rotation2d(Math.PI);

        midPose = new Pose2d(2.54, 0, rot);
        startPose = new Pose2d(2.54, 0, startRot);
        endPose = new Pose2d(5.5, 0, rot);
        endPoseNoRotate = new Pose2d(5.5, 0, startRot);
        initialPose = new Pose2d(-.5,0, startRot);//it is -0.5 because for some reason when it is 0 it doesnt come back all the way
        poseRight = new Pose2d(5, -1.75, startRot);
        poseCharge = new Pose2d(2.54, -1.5, startRot);

        addCommands(
            //deposit
            //new GoToPose(m_drive, startPose),
            //new WaitCommand(2),
            //new GoToPose(m_drive, midPose),
            //new WaitCommand(2),
            new GoToPose(m_drive, endPoseNoRotate), //it goes to the place we want it to
            new GoToPose(m_drive, poseRight),
            new GoToPose(m_drive, poseCharge),
            new BalanceCommand(m_drive)
            //new WaitCommand(2),
            //intake
            //new GoToPose(m_drive, startPose)// it goes back to start position

        );
    }
}
