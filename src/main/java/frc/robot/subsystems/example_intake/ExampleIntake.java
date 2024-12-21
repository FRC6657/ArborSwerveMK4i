// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.example_intake;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExampleIntake extends SubsystemBase {

  double rotation = 90;

  public Command up() {
    return this.runOnce(
        () -> {
          rotation = 152;
        }).andThen(Commands.print("Up"));
  }

  public Command down() {
    return this.runOnce(
        () -> {
          rotation = -19;
        }).andThen(Commands.print("Down"));
  }

  public Pose3d getMechanismPose() {
    return new Pose3d(
        0.332169, 0, 0.210783, new Rotation3d(0, -Units.degreesToRadians(rotation + 19), 0));
  }
}
