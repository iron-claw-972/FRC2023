package frc.robot.util;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.vision.CalculateStdDevs;
import frc.robot.commands.vision.TestVisionAlignment;
import frc.robot.commands.vision.TestVisionDistance;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.Drivetrain;

public class Vision {
  // The field layout
  private AprilTagFieldLayout m_aprilTagFieldLayout;
  // A list of the cameras on the robot
  private ArrayList<VisionCamera> m_cameras = new ArrayList<>();
  private ShuffleboardTab m_shuffleboardTab;

  /**
   * Creates a new instance of Vision
   * Sets up field layout, and cameras
   * @param shuffleboardTab The vision shuffleboard tab
   * @param camList The list of camera names and their translation from the center of the robot
   */
  public Vision(ShuffleboardTab shuffleboardTab, List<Pair<String, Transform3d>> camList) {
    m_shuffleboardTab = shuffleboardTab;

    try {
      // Try to find the field layout
      m_aprilTagFieldLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
    } catch (IOException e) {
      // If it can't find it, use the layout in the constants
      m_aprilTagFieldLayout = new AprilTagFieldLayout(FieldConstants.kAprilTags, FieldConstants.kFieldLength, FieldConstants.kFieldWidth);
      DriverStation.reportWarning("Could not find k2023ChargedUp.m_resourceFile, check that GradleRIO is updated to at least 2023.2.1 in build.gradle",  e.getStackTrace());
    }
    // Sets the origin to the right side of the blue alliance wall
    m_aprilTagFieldLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);

    // Puts the cameras in an array list
    for (int i = 0; i < camList.size(); i++) {
      m_cameras.add(this.new VisionCamera(camList.get(i).getFirst(), camList.get(i).getSecond()));
    }
  }

  /**
   * Returns where it thinks the robot is
   * @param referencePose The pose to use as a reference, usually the previous robot pose
   * @return An array list of estimated poses, one for each camera that can see an april tag
   */
  public ArrayList<EstimatedRobotPose> getEstimatedPoses(Pose2d referencePose) {
    ArrayList<EstimatedRobotPose> estimatedPoses = new ArrayList<>();
    for (int i = 0; i < m_cameras.size(); i++) {
      Optional<EstimatedRobotPose> estimatedPose = m_cameras.get(i).getEstimatedPose(referencePose);
      // If the camera can see an april tag that exists, add it to the array list
      // April tags that don't exist might return a result that is present but doesn't have a pose
      if (estimatedPose.isPresent() && estimatedPose.get().estimatedPose!=null) {
        estimatedPoses.add(estimatedPose.get());
        LogManager.addDoubleArray("Vison/camera " + i + "/estimated pose2d", new double[] {
          estimatedPose.get().estimatedPose.getX(),
          estimatedPose.get().estimatedPose.getY(),
          estimatedPose.get().estimatedPose.getRotation().getZ()
        });
      }
    }
    return estimatedPoses;
  }

  /**
   * Gets the pose as a Pose2d
   * @param referencePoses The reference poses in order of preference, null poses will be skipped
   * @return The pose of the robot, or null if it can't see april tags
   */
  public Pose2d getPose2d(Pose2d... referencePoses){
    Pose2d referencePose = new Pose2d();
    for (Pose2d checkReferencePose:referencePoses){
      if (checkReferencePose != null) {
        referencePose = checkReferencePose;
        break;
      }
    }
    ArrayList<EstimatedRobotPose> estimatedPoses = getEstimatedPoses(referencePose);
    Translation2d translation = new Translation2d();
    double rotation = 0;
    
    if (estimatedPoses.size() == 1) return estimatedPoses.get(0).estimatedPose.toPose2d();
    
    if (estimatedPoses.size() == 2) {
      return new Pose2d(
        estimatedPoses.get(0).estimatedPose.toPose2d().getTranslation()
          .plus(estimatedPoses.get(1).estimatedPose.toPose2d().getTranslation())
          .div(2),
          
          new Rotation2d(Functions.modulusMidpoint(
            estimatedPoses.get(0).estimatedPose.toPose2d().getRotation().getRadians(),
            estimatedPoses.get(1).estimatedPose.toPose2d().getRotation().getRadians(),
            -Math.PI, Math.PI
          )
        )
      );
    }
          
    //TODO: VERY LOW PRIORITY FOR FUTURE ROBOTS, make the rotation average work with more than 2 cameras
    // for(int i = 0; i < estimatedPoses.size(); i ++){
    //   translation=translation.plus(estimatedPoses.get(i).estimatedPose.toPose2d().getTranslation());
    // }

    // if(posesUsed>0){
    //   return new Pose2d(translation.div(estimatedPoses.size()), new Rotation2d());
    // }
    return null;
  }

  public AprilTagFieldLayout getAprilTagFieldLayout(){
    return m_aprilTagFieldLayout;
  }

  /**
   * Gets the pose of an april tag
   * @param id AprilTag id (1-8)
   * @return Pose3d of the AprilTag
   */
  public Pose3d getTagPose(int id){
    if(id < 1 || id > 8){
      System.out.println("Tried to find the pose of april tag "+id);
      return null;
    }
    return getAprilTagFieldLayout().getTagPose(id).get();
  }
  
  public void setupVisionShuffleboard() {
  }
  
  class VisionCamera {
    PhotonCamera camera;
    PhotonPoseEstimator photonPoseEstimator;
  
    /**
     * Stores information about a camera
     * @param cameraName The name of the camera on PhotonVision
     * @param robotToCam The transformation from the robot to the camera
     */
    public VisionCamera(String cameraName, Transform3d robotToCam) {
      camera = new PhotonCamera(cameraName);
      photonPoseEstimator = new PhotonPoseEstimator(
        m_aprilTagFieldLayout, 
        PoseStrategy.MULTI_TAG_PNP, 
        camera, 
        robotToCam
      );
      photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
      photonPoseEstimator.setReferencePose(new Pose2d());
    }
  
    /**
     * Gets the estimated pose from the camera
     * @param referencePose Pose to use for reference, usually the previous estimated robot pose
     * @return estimated robot pose
     */
    public Optional<EstimatedRobotPose> getEstimatedPose(Pose2d referencePose) {
      photonPoseEstimator.setReferencePose(referencePose);
      Optional<EstimatedRobotPose> pose = photonPoseEstimator.update();
      
      // if there is a pose, check the ambiguity isn't too high
      if (pose.isPresent()) {
        // go through all the targets
        List<PhotonTrackedTarget> targetsUsed = pose.get().targetsUsed;
        for (int i = 0; i < targetsUsed.size(); i++) {
          // check their ambiguity, if it is above the highest wanted amount, return nothing
          if (targetsUsed.get(i).getPoseAmbiguity() > VisionConstants.highestAmbiguity) {
            return Optional.empty();
          }
        }
      }
      return pose;
    }
  }

  public void addTestCommands(ShuffleboardTab testTab, GenericEntry testEntry, Drivetrain drive){
    testTab.add("Calculate vision std devs", new CalculateStdDevs(1000, drive, this).beforeStarting(new WaitCommand(5)));
    testTab.add("Test vision (forward)", new TestVisionDistance(0.2, drive, this));
    testTab.add("Test vision (backward)", new TestVisionDistance(-0.2, drive, this));
    testTab.add("Align to 0 degrees", new TestVisionAlignment(0, drive, this));
    testTab.add("Align to 90 degrees", new TestVisionAlignment(Math.PI/2, drive, this));
    testTab.add("Align to -90 degrees", new TestVisionAlignment(-Math.PI/2, drive, this));
    testTab.add("Align to 180 degrees", new TestVisionAlignment(Math.PI, drive, this));
  }
}
