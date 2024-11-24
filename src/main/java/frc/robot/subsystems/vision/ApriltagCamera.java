// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.Constants.VisionConstants.CameraInfo;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class ApriltagCamera {

  private final ApriltagCameraIO io;
  private final AprilTagCameraIOInputsAutoLogged inputs = new AprilTagCameraIOInputsAutoLogged();

  private final PhotonPoseEstimator poseEstimator;
  private final CameraInfo cameraInfo;

  private Pose3d latestPose = new Pose3d();

  public ApriltagCamera(ApriltagCameraIO io, CameraInfo cameraInfo) {
    this.io = io;
    this.cameraInfo = cameraInfo;

    poseEstimator =
        new PhotonPoseEstimator(
            AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo),
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            cameraInfo.robotToCamera);
  }

  public void updateInputs() {
    io.updateInputs(inputs);

    var result = poseEstimator.update(inputs.result);

    if (result.isPresent()) {
      latestPose = result.get().estimatedPose;
    } else {
      latestPose = new Pose3d();
    }

    Logger.processInputs("Vision/ApriltagCameras/" + cameraInfo.cameraName + "/", inputs);
    Logger.recordOutput("Vision/ApriltagCameras/" + cameraInfo.cameraName + "/Pose", latestPose);
  }

  public Pose3d getEstimatedPose() {
    return latestPose;
  }
}
