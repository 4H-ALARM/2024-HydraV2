package frc.lib.configs;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.PhotonPoseEstimator;

public class cameraConfig {
    public final String photonCamera;
    public final AprilTagFieldLayout aprilTagFieldLayout;
    public final Transform3d robotToCam;
    public final PhotonPoseEstimator.PoseStrategy photonPoseEstimatorStrategy;

    public cameraConfig(String photonCamera1key, AprilTagFieldLayout aprilTagFieldLayout, Transform3d robotToCam, PhotonPoseEstimator.PoseStrategy poseEstimator) {
        this.photonCamera = photonCamera1key;
        this.aprilTagFieldLayout = aprilTagFieldLayout;
        this.robotToCam = robotToCam;
        this.photonPoseEstimatorStrategy = poseEstimator;
    }




}
