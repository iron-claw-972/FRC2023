
package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.BalanceCommand;
import frc.robot.commands.GoToPose;
import frc.robot.subsystems.Drivetrain;

public class EngageFromRightDriverSide extends SequentialCommandGroup{

    private Drivetrain m_drive;
    private Pose2d leftUnrotatedPose, chargePose, centerPose, leftRotatedPose;
    private Rotation2d rot, startRot;

    public EngageFromRightDriverSide(Drivetrain drive)  {
        //DRIVES OUT OF COMMUNITY, THEN MOVES TO RIGHT AND BACK TO ENGAGE. THIS PATH IS FOR THE GRID POSITION CLOSEST TO BARRIER
        m_drive = drive;

        startRot = new Rotation2d(0);
        rot = new Rotation2d(Math.PI);

        centerPose = new Pose2d(5.5, 0, startRot);
        leftUnrotatedPose = new Pose2d(5, 1.75, startRot);
        chargePose = new Pose2d(2.54, -1.5, startRot);
        leftRotatedPose = new Pose2d(5, 1.75, rot);


        addCommands(
            new GoToPose(m_drive, centerPose),
            new GoToPose(m_drive, leftUnrotatedPose),//alternatively could make it poseRightRotated, but this is untested
            new GoToPose(m_drive, chargePose),
            new BalanceCommand(m_drive)
            );
        }
    }
    