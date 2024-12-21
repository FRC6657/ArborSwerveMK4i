package frc.robot.subsystems;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.subsystems.drivebase.Swerve;
import frc.robot.subsystems.example_intake.ExampleIntake;
import org.littletonrobotics.junction.Logger;

public class Superstructure {

  Swerve drivebase;
  ExampleIntake intake;

  public Superstructure(Swerve drivebase, ExampleIntake intake) {
    this.drivebase = drivebase;
    this.intake = intake;
  }

  public void update3DPose() {
    Pose3d[] mechanismPoses = new Pose3d[4];
    mechanismPoses[0] = new Pose3d();
    mechanismPoses[1] = intake.getMechanismPose();
    mechanismPoses[2] = new Pose3d();
    mechanismPoses[3] = new Pose3d();
    Logger.recordOutput("3D Poses", mechanismPoses);
  }

  // Simple Test Auto that just runs a path.
  public AutoRoutine testAuto(AutoFactory factory) {

    final AutoRoutine routine = factory.newRoutine("Test");

    final AutoTrajectory testPath = routine.trajectory("Test");

    routine
        .active()
        .onTrue(
            drivebase
                .resetOdometry(
                    testPath
                        .getInitialPose()
                        .orElseGet(
                            () -> {
                              routine.kill();
                              return new Pose2d();
                            }))
                .andThen(testPath.cmd()));

    return routine;
  }
}
