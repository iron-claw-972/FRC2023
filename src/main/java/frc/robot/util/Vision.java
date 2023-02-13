/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

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
import frc.robot.constants.FieldConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.Drivetrain;



public class Vision {
  private static RobotPoseEstimator robotPoseEstimator;
  // private static RobotPoseEstimator robotPoseEstimator2;
  private static AprilTagFieldLayout aprilTagFieldLayout;
  private static Drivetrain drive;
 

  /**
   * Sets up the cameras and pose estimator
   * @param drive The drivetrain
   */
  public static void setup(Drivetrain drive) {
    setup(drive, new PhotonCamera(VisionConstants.kCameraName1), new PhotonCamera(VisionConstants.kCameraName2));
  }

  /**
   * Sets up the cameras and pose estimator
   * @param drive The drivetrain
   * @param camera1 The first camera
   * @param camera2 The second camera
   */
  public static void setup(Drivetrain drive, PhotonCamera camera1, PhotonCamera camera2) {
    Vision.drive=drive;
    ArrayList<Pair<PhotonCamera, Transform3d>> camList;
    if(VisionConstants.k2Cameras){
      camList = new ArrayList<Pair<PhotonCamera, Transform3d>>(List.of(
        new Pair<PhotonCamera, Transform3d>(camera1, VisionConstants.kRobotToCamera1),
        new Pair<PhotonCamera, Transform3d>(camera2, VisionConstants.kRobotToCamera2)
      ));
    }else{
      camList = new ArrayList<Pair<PhotonCamera, Transform3d>>(List.of(
        new Pair<PhotonCamera, Transform3d>(camera1, VisionConstants.kRobotToCamera1)
      ));
    }
    getTagFieldLayout();
    aprilTagFieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);
    
    robotPoseEstimator = new RobotPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camList);

    if(VisionConstants.k2Cameras){
      // ArrayList<Pair<PhotonCamera, Transform3d>>camList = new ArrayList<Pair<PhotonCamera, Transform3d>>(List.of(
      //   new Pair<PhotonCamera, Transform3d>(camera2, VisionConstants.kCameraToRobot2)
      // ));
      // robotPoseEstimator2 = new RobotPoseEstimator(aprilTagFieldLayout, PoseStrategy.AVERAGE_BEST_TARGETS, camList);    
    }

  }

  public static AprilTagFieldLayout getTagFieldLayout() {
    // AprilTagFields.loadAprilTagLayoutField();
    try {
      //TODO: Fix this
      // aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
      aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2022RapidReact.m_resourceFile);
    } catch (IOException ex) {
      aprilTagFieldLayout = new AprilTagFieldLayout(VisionConstants.kTagPoses, FieldConstants.kFieldLength, FieldConstants.kFieldWidth);
      System.out.println("Vision setup IOException: "+ex.getMessage());
    }

    return aprilTagFieldLayout;
  }

  // public Map<Pose3d, Double> getPoseEstimation() {
  //   if (m_hasTargets) {
  //     double imageCaptureTime = Timer.getFPGATimestamp() - m_latency;
  //     Transform3d camToTargetTrans = m_bestTarget.getBestCameraToTarget();
  //     Pose3d camPose = VisionConstants.kTagPoses.get(m_targetId).transformBy(camToTargetTrans.inverse());
  //     return Collections.singletonMap(camPose.transformBy(VisionConstants.kCameraToRobot), imageCaptureTime);
  //   }
  //   return null;
  // }

  /**
   * Prints the estimated pose from the vision
   */
  public static void printEstimate(){
    robotPoseEstimator.setReferencePose(new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0)));
    Optional<Pair<Pose3d,Double>> r = robotPoseEstimator.update();
    if(r.isPresent()){
      System.out.printf("Present: %b\nPose: (%.2f, %.2f, %.2f)\nRotation: %.2f degrees\nTime: %.2f\n",
        r.isPresent(), 
        r.get().getFirst().getX(), 
        r.get().getFirst().getY(), 
        r.get().getFirst().getZ(), 
        r.get().getFirst().getRotation().toRotation2d().getDegrees(), 
        r.get().getSecond()
      );
    }else{
      System.out.println("Result is not present");
    }
  }

  public static Optional<Pair<Pose3d,Double>> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    robotPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    Optional<Pair<Pose3d,Double>> r = robotPoseEstimator.update();
    return r;

    // double currentTime = Timer.getFPGATimestamp();
    // Optional<Pair<Pose3d, Double>> result = m_robotPoseEstimator.update();
    // if (result.isPresent()) {
    //     return new Pair<Pose2d, Double>(result.get().getFirst().toPose2d(), currentTime - result.get().getSecond());
    // } else {
    //     return new Pair<Pose2d, Double>(null, 0.0);
    // }
}

public static Pose2d getPose2d(Pose2d referencePose){
  Optional<Pair<Pose3d, Double>> p = getEstimatedGlobalPose(referencePose==null?drive.getPose():referencePose);
  if(p.isPresent() && p.get().getFirst() != null && p.get().getSecond() != null && p.get().getFirst().getX() > -10000 && p.get().getSecond() >= 0){
    return p.get().getFirst().toPose2d();
  }
  return null;
}

/**
 * Not used
 * @param prevEstimatedRobotPose Doesn't do anything
 * @return null
 */
public static Optional<Pair<Pose3d,Double>> getEstimatedGlobalPose2(Pose2d prevEstimatedRobotPose) {
  // robotPoseEstimator2.setReferencePose(prevEstimatedRobotPose);
  // return robotPoseEstimator2.update();
  return null;

  // double currentTime = Timer.getFPGATimestamp();
  // Optional<Pair<Pose3d, Double>> result = m_robotPoseEstimator.update();
  // if (result.isPresent()) {
  //     return new Pair<Pose2d, Double>(result.get().getFirst().toPose2d(), currentTime - result.get().getSecond());
  // } else {
  //     return new Pair<Pose2d, Double>(null, 0.0);
  // }
}

  public static AprilTagFieldLayout getAprilTagFieldLayout(){
    return aprilTagFieldLayout;
  }

  public static Pose3d getTagPose(int id){
    return getAprilTagFieldLayout().getTagPose(id).get();
  }
}
