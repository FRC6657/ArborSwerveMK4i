package frc.robot.subsystems.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import frc.robot.Constants.VisionConstants.CameraInfo;

public class ApriltagCameraIO_Real implements ApriltagCameraIO {

  PhotonCamera camera;

  public ApriltagCameraIO_Real(CameraInfo cameraInfo) {
    camera = new PhotonCamera(cameraInfo.cameraName);
  }

  @Override
  public void updateInputs(AprilTagCameraIOInputs inputs) {
    var results = camera.getAllUnreadResults();
    if (!results.isEmpty()) {
      inputs.result = results.get(results.size() - 1);
    } else {
      inputs.result = new PhotonPipelineResult();
    }
  }
}
