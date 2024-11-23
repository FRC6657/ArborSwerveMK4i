// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

public class ApriltagCamera {

  private final ApriltagCameraIO io;
  private final ApriltagCameraIOInputsLogged inputs = new ApriltagCameraIOInputsLogged();

  public ApriltagCamera(ApriltagCameraIO io) {
    this.io = io;
  }

  public void setSimPose(
      Optional<EstimatedRobotPose> simEst, ApriltagCamera camera, boolean newResult) {
    this.io.setSimPose(simEst, camera, newResult);
  }

  public void updateInputs() {
    io.updateInputs(inputs);
  }

  public Optional<EstimatedRobotPose> update(PhotonPipelineResult result) {
    // Skip if we only have 1 target
    // TODO change
    if (result.getTargets().size() < 1) {
      return Optional.empty();
    }
    var estPose =
        VisionHelper.update(
            result,
            inputs.constants.intrinsicsMatrix(),
            inputs.constants.distCoeffs(),
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            inputs.constants.robotToCamera(),
            inputs.pnpTransform);
    // // Reject if estimated pose is in the air or ground
    if (estPose.isPresent() && Math.abs(estPose.get().estimatedPose.getZ()) > 0.25) {
      return Optional.empty();
    }
    return estPose;
  }

  public String getName() {
    return io.getName();
  }
}
