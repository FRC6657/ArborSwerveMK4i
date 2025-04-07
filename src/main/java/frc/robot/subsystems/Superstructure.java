package frc.robot.subsystems;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.swerve.Swerve;

public class Superstructure {

  Swerve drivebase;

  public Superstructure(Swerve drivebase) {
    this.drivebase = drivebase;
  }

  // Simple Test Auto that just runs a path.
  public AutoRoutine testAuto(AutoFactory factory) {
    final AutoRoutine routine = factory.newRoutine("Test");
    final AutoTrajectory testPath = routine.trajectory("Test");
    routine.active().onTrue(Commands.sequence(testPath.resetOdometry(), testPath.cmd()));
    return routine;
  }
}
