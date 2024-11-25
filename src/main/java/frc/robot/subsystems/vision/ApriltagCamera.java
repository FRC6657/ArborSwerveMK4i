// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.VisionConstants.CameraInfo;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

public class ApriltagCamera {

  private final ApriltagCameraIO io;
  private final AprilTagCameraIOInputsAutoLogged inputs = new AprilTagCameraIOInputsAutoLogged();

  private final PhotonPoseEstimator poseEstimator;
  private final CameraInfo cameraInfo;

  private Pose3d latestPose = new Pose3d();
  private Matrix<N3, N1> stdDevs =
      VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
  private double latestTimestamp = -1;

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
      latestTimestamp = result.get().timestampSeconds;
      stdDevs = getEstimationStdDevs(latestPose.toPose2d(), inputs.result);

    } else {
      latestPose = new Pose3d(new Translation3d(100, 100, 100), new Rotation3d());
      stdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    }

    Logger.processInputs("Vision/ApriltagCameras/" + cameraInfo.cameraName + "/", inputs);
    Logger.recordOutput("Vision/ApriltagCameras/" + cameraInfo.cameraName + "/Pose", latestPose);
  }

  public Pose3d getEstimatedPose() {
    return latestPose;
  }

  private Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose, PhotonPipelineResult result) {
    var estStdDevs = VisionConstants.singleTagStdDev;
    var targets = result.targets;
    int numTags = 0;
    double avgDist = 0;
    for (var tgt : targets) {
      var tagPose = poseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
      if (tagPose.isEmpty()) continue;
      numTags++;
      avgDist +=
          tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
    }
    if (numTags == 0) return estStdDevs;
    avgDist /= numTags;
    // Decrease std devs if multiple targets are visible
    if (numTags > 1) estStdDevs = VisionConstants.multiTagStdDev;
    // Increase std devs based on (average) distance
    if (numTags == 1 && avgDist > 4)
      estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

    return estStdDevs;
  }

  public Matrix<N3, N1> getLatestStdDevs() {
    return stdDevs;
  }

  public double getLatestTimestamp() {
    return latestTimestamp;
  }

  public void updateSimPose(Pose2d robotPose) {
    io.updateSimPose(robotPose);
  }
}
