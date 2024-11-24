package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;
import org.photonvision.targeting.PhotonPipelineResult;

public interface ApriltagCameraIO {

  @AutoLog
  public static class AprilTagCameraIOInputs {
    public PhotonPipelineResult result = new PhotonPipelineResult();
  }

  public default void updateInputs(AprilTagCameraIOInputs inputs) {}
}
