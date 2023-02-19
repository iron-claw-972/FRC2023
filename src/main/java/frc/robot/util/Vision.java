package frc.robot.util;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.VisionConstants;

public class Vision {
  private AprilTagFieldLayout m_aprilTagFieldLayout;
  private ArrayList<VisionCamera> m_cameras = new ArrayList<>();
  private ShuffleboardTab m_shuffleboardTab;

  /**
   * Sets up the cameras and pose estimator
   * @param drive The drivetrain
   * @param m_cameras The list of camera names and their translation from the center of robot
   */
  public Vision(ShuffleboardTab shuffleboardTab, List<Pair<String, Transform3d>> camList) {
    m_shuffleboardTab = shuffleboardTab;

    try {
      m_aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
    } catch (IOException ex) {
      m_aprilTagFieldLayout = new AprilTagFieldLayout(VisionConstants.kAprilTags, FieldConstants.kFieldLength, FieldConstants.kFieldWidth);
      System.out.println("Could not find k2023ChargedUp.m_resourceFile: " + ex.getMessage());
    }
    m_aprilTagFieldLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);

    for (int i = 0; i < camList.size(); i++) {
      m_cameras.add(this.new VisionCamera(camList.get(i).getFirst(), camList.get(i).getSecond()));
    }
  }

  public ArrayList<EstimatedRobotPose> getEstimatedPoses(Pose2d referencePose) {
    ArrayList<EstimatedRobotPose> estimatedPoses = new ArrayList<>();
    for (int i = 0; i < m_cameras.size(); i++) {
      Optional<EstimatedRobotPose> estimatedPose = m_cameras.get(i).getEstimatedPose(referencePose);
      if (estimatedPose.isPresent()) {
        estimatedPoses.add(estimatedPose.get());
      }
    }
    return estimatedPoses;
  }

  public AprilTagFieldLayout getAprilTagFieldLayout(){
    return m_aprilTagFieldLayout;
  }

  public Pose3d getTagPose(int id){
    return getAprilTagFieldLayout().getTagPose(id).get();
  }

  class VisionCamera {
    PhotonCamera camera;
    PhotonPoseEstimator photonPoseEstimator;
  
    public VisionCamera(String cameraName, Transform3d robotToCam) {
      camera = new PhotonCamera(cameraName);
      photonPoseEstimator = new PhotonPoseEstimator(m_aprilTagFieldLayout, PoseStrategy.AVERAGE_BEST_TARGETS, camera, robotToCam);
    }
  
    /**
     * @param referencePose previous estimated robot pose
     * @return estimated robot pose
     */
    public Optional<EstimatedRobotPose> getEstimatedPose(Pose2d referencePose) {
      photonPoseEstimator.setReferencePose(referencePose);
      return photonPoseEstimator.update();
    }
  }
}
