package frc.robot.subsystems.vision;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants.VisionConstants;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;

public interface ApriltagCameraIO {

  public static class ApriltagCameraIOInputs {
    public double[] timestamps = new double[] {};
    public double[] latencies = new double[] {};
    public List<PhotonTrackedTarget> targets = new ArrayList<>();
    public int numTags = 0;
    public Transform3d pnpTransform = new Transform3d();
    public Pose3d[] targetPoses = new Pose3d[] {};
    public VisionConstants constants =
        new VisionConstants(
            "Default",
            new Transform3d(),
            Matrix.eye(Nat.N3()),
            MatBuilder.fill(Nat.N8(), Nat.N1(), 0.0, 0.0, 0.0, 0.0, 0.0),
            new int[] {1280, 800});
  }

  public default void updateInputs(ApriltagCameraIOInputs inputs) {}

  public default void setSimPose(
      Optional<EstimatedRobotPose> simEst, ApriltagCamera camera, boolean newResult) {}

  public default String getName() {
    return "Unset";
  }
}
