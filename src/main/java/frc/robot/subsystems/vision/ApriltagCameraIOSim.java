package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.Constants.VisionConstants;
import java.util.Optional;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class ApriltagCameraIOSim implements ApriltagCameraIO {

  private final VisionSystemSim sim;
  private final PhotonCamera camera;
  private final PhotonCameraSim cameraSim;
  private final VisionConstants constants;
  public static Supplier<Pose3d> pose;

  public ApriltagCameraIOSim(VisionConstants constants) {
    this.sim = new VisionSystemSim(constants.cameraName());
    var cameraProp = new SimCameraProperties();
    cameraProp.setCalibration(
        constants.cameraRes()[0],
        constants.cameraRes()[1],
        constants.intrinsicsMatrix(),
        constants.distCoeffs());
    cameraProp.setCalibError(0, 0);
    cameraProp.setFPS(50);
    cameraProp.setAvgLatencyMs(50.0);
    cameraProp.setLatencyStdDevMs(5.0);
    this.camera = new PhotonCamera(constants.cameraName());
    this.cameraSim = new PhotonCameraSim(camera, cameraProp);
    cameraSim.enableDrawWireframe(true);
    cameraSim.setMaxSightRange(7);
    this.constants = constants;
    sim.addCamera(cameraSim, constants.robotToCamera());

    try {
      var field = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);
      field.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
      sim.addAprilTags(field);
    } catch (Exception e) {
      e.printStackTrace();
    }
  }

  @Override
  public void setSimPose(
      Optional<EstimatedRobotPose> simEst, ApriltagCamera camera, boolean newResult) {
    simEst.ifPresentOrElse(
        est ->
            VisionHelper.getSimDebugField(sim)
                .getObject("VisionEstimation")
                .setPose(est.estimatedPose.toPose2d()),
        () -> {
          if (newResult)
            VisionHelper.getSimDebugField(sim).getObject("VisionEstimation").setPoses();
        });
  }

  @Override
  public void updateInputs(ApriltagCameraIOInputs inputs) {
    sim.update(pose.get().toPose2d());

    var results = camera.getAllUnreadResults();

    if (!results.isEmpty()) {

      inputs.timestamps = new double[results.size()];
      inputs.latencies = new double[results.size()];

      for (int i = 0; i < results.size(); i++) {
        inputs.timestamps[i] = results.get(i).getTimestampSeconds();
        inputs.latencies[i] = results.get(i).metadata.getLatencyMillis();
        inputs.targets = results.get(0).targets;
      }
    }
  }

  @Override
  public String getName() {
    return constants.cameraName();
  }
}
