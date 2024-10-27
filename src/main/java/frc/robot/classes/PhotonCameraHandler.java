package frc.robot.classes;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.lib.configs.cameraConfig;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import java.util.Optional;
import java.util.Vector;

public class PhotonCameraHandler {
    private final PhotonCamera photonCamera;
    private final PhotonPoseEstimator photonPoseEstimator;
    private final cameraConfig config;


    public PhotonCameraHandler(cameraConfig config) {
        this.config = config;

        this.photonCamera = new PhotonCamera(config.photonCamera);
        this.photonPoseEstimator = new PhotonPoseEstimator(this.config.aprilTagFieldLayout, this.config.photonPoseEstimatorStrategy, this.photonCamera, this.config.robotToCam);
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        if (photonPoseEstimator.update().isPresent()) {
            Logger.recordOutput("Cameras/"+this.config.photonCamera+"/pose", photonPoseEstimator.update().get().estimatedPose.toPose2d());
        }
        Logger.recordOutput("Cameras/"+this.config.photonCamera+"/isConnected", photonCamera.isConnected());
        return photonPoseEstimator.update();
    }



    public PhotonCamera getPhotonCamera() {
        return photonCamera;
    }
}
