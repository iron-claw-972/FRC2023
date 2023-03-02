package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.GoToPose;
import frc.robot.subsystems.Drivetrain;

public class ToCenterAndBack extends SequentialCommandGroup{

    private Drivetrain m_drive;
    private Pose2d centerPose, initialPose;
    private Rotation2d rot, startRot;

    /**
     * Drives to center line, then drives back. Can be done on either side of drivestation. 
     * @param drive drivetrain
     */
    public ToCenterAndBack(Drivetrain drive)  {
        m_drive = drive;
        
        startRot = new Rotation2d(0);
        rot = new Rotation2d(Math.PI);

        centerPose = new Pose2d(5.5, 0, rot);
        initialPose = new Pose2d(-.5,0, startRot);//it is -0.5 to account for significant error in which it did not come back all the way

        addCommands(
            new GoToPose(m_drive, centerPose), 
            new GoToPose(m_drive, initialPose)
           
        );
    }
}

