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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
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
      m_aprilTagFieldLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
    } catch (IOException e) {
      m_aprilTagFieldLayout = new AprilTagFieldLayout(VisionConstants.kAprilTags, VisionConstants.kFieldLength, VisionConstants.kFieldWidth);
      DriverStation.reportWarning("Could not find k2023ChargedUp.m_resourceFile, check that GradleRIO is updated to at least 2023.2.1 in build.gradle",  e.getStackTrace());
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

  public Pose2d getPose2d(Pose2d referencePose, Pose2d robotPose){
    ArrayList<EstimatedRobotPose> p = getEstimatedPoses(referencePose==null?robotPose:referencePose);
    Pose2d p2;
    if(p.size()==0){
      return null;
    }else if(p.size()==1){
      p2=p.get(0).estimatedPose.toPose2d();
    }else{
      p2 = new Pose2d(p.get(0).estimatedPose.getX()/2+p.get(1).estimatedPose.getX()/2, p.get(0).estimatedPose.getY()/2+p.get(1).estimatedPose.getY()/2, new Rotation2d(p.get(0).estimatedPose.toPose2d().getRotation().getRadians()/2+p.get(1).estimatedPose.toPose2d().getRotation().getRadians()/2));
    }
    return p2;
  }

  public AprilTagFieldLayout getAprilTagFieldLayout(){
    return m_aprilTagFieldLayout;
  }

  /**
   * @param id AprilTag id (1-8)
   * @return Pose3d of the AprilTag
   */
  public Pose3d getTagPose(int id){
    return getAprilTagFieldLayout().getTagPose(id).get();
  }

  public void setupVisionShuffleboard() {
  }

  class VisionCamera {
    PhotonCamera camera;
    PhotonPoseEstimator photonPoseEstimator;
  
    public VisionCamera(String cameraName, Transform3d robotToCam) {
      camera = new PhotonCamera(cameraName);
      photonPoseEstimator = new PhotonPoseEstimator(m_aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP, camera, robotToCam);
      photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
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
