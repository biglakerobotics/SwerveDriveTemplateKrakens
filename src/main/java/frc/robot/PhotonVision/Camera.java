package frc.robot.PhotonVision;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

public class Camera {
    public PhotonCamera camera;
    public PhotonPoseEstimator pose;

    public Camera(PhotonCamera camera, PhotonPoseEstimator pose) {
        this.camera = camera;
        this.pose = pose;
    }
    

}
