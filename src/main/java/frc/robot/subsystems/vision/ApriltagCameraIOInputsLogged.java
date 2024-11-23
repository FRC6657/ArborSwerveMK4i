package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ApriltagCameraIOInputsLogged extends ApriltagCameraIO.ApriltagCameraIOInputs
    implements LoggableInputs, Cloneable {

  @Override
  public void toLog(LogTable table) {

    table.put("Timestamps", timestamps);
    table.put("Latencies", latencies);

    targetPoses = new Pose3d[targets.size()];
    for (int i = 0; i < targets.size(); i++) {
      VisionHelper.logPhotonTrackedTarget(targets.get(i), table, String.valueOf(i));
      targetPoses[i] = VisionHelper.fieldTags.getTagPose(targets.get(i).getFiducialId()).get();
    }
    table.put("Tag Count", targets.size());
    table.put("Pose", pnpTransform);
    table.put("Target Poses", targetPoses);
  }

  @Override
  public void fromLog(LogTable table) {
    timestamps = table.get("Timestamps", timestamps);
    latencies = table.get("Latencies", latencies);
    for (int i = 0; i < table.get("NumTags", numTags); i++) {
      this.targets.add(VisionHelper.getLoggedPhotonTrackedTarget(table, String.valueOf(i)));
    }
    numTags = table.get("NumTags", numTags);
    pnpTransform = table.get("Pose", pnpTransform);
    targetPoses = table.get("Target Pose3ds", targetPoses);
    constants = VisionHelper.getLoggedVisionConstants(table);
  }

  public ApriltagCameraIOInputsLogged clone() {
    ApriltagCameraIOInputsLogged copy = new ApriltagCameraIOInputsLogged();
    copy.timestamps = this.timestamps;
    copy.latencies = this.latencies;
    copy.targets = this.targets;
    copy.numTags = this.numTags;
    copy.pnpTransform = this.pnpTransform;
    copy.targetPoses = this.targetPoses;
    return copy;
  }
}
