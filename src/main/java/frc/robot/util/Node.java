package frc.robot.util;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.RobotPoseEstimator;
import org.photonvision.RobotPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.Robot;
import frc.robot.Robot.Teams;

/**Class to store data about scoring locations*/
public class Node {
    // Possible node types
    public enum Types {CONE, CUBE, HYBRID};

    public final Teams team;
    public final int row;
    public final int x;
    public final Pose3d pose;
    public final Pose2d scorePose;
    public final Types type;

    /**
     * Creates a new Node object
     * @param team
     *  The node's color
     * @param row
     *  Which row it's in (1 = bottom, 2 = middle, 3 = top)
     * @param x
     *  X position of the node from the alliance area (1-9)
     */
    public Node(Teams team, int row, int x){
        this.team=team;
        this.row=row;
        this.x=x;
        type = row==1?Types.HYBRID:x%3==2?Types.CUBE:Types.CONE;

        Pose3d tag = Robot.aprilTags[(x-1)/3+(team==Teams.RED?1:6)];
        double x2;
        double z;
        switch(row){
            case(1):
            x2 = tag.getX()+Units.inchesToMeters(14.25/2)*(team==Teams.RED?-1:1);
            z = 0;
            break;
            case(2):
            x2 = tag.getX()-Units.inchesToMeters((31.625-14.25)/2)*(team==Teams.RED?-1:1);
            z = Units.inchesToMeters(type==Types.CUBE?23.5:34);
            break;
            case(3):
            x2 = tag.getX()-Units.inchesToMeters(39.75-14.25)*(team==Teams.RED?-1:1);
            z = Units.inchesToMeters(type==Types.CUBE?35.5:46);
            break;
            default:
            throw(new IllegalArgumentException("Row had to be 1, 2, or 3"));
        }
        double y = tag.getY()+(x%3==2?0:Units.inchesToMeters(22))*((x%3==0)^(team==Teams.RED)?-1:1);
        pose=new Pose3d(x2, y, z, tag.getRotation());
        scorePose=new Pose2d(tag.getX()+(15.25+26/2)*(team==Teams.RED?-1:1), y, new Rotation2d(team==Teams.RED?0:Math.PI));
    }
}