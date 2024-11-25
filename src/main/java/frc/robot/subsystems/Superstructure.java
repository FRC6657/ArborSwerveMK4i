package frc.robot.subsystems;

import choreo.auto.AutoFactory;
import choreo.auto.AutoLoop;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.drivebase.Swerve;

public class Superstructure {

  Swerve drivebase;

  public Superstructure(Swerve drivebase) {
    this.drivebase = drivebase;
  }

  public AutoLoop testAuto(AutoFactory factory) {

    final AutoLoop routine = factory.newLoop("Test Auto");

    final AutoTrajectory testPath = factory.trajectory("TestPath", routine);

    routine
        .enabled()
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
