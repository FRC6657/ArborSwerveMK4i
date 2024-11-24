// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivebase;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Swerve extends SubsystemBase {

  private Module[] modules =
      new Module[] {
        new Module(new ModuleIO() {}, ""),
        new Module(new ModuleIO() {}, ""),
        new Module(new ModuleIO() {}, ""),
        new Module(new ModuleIO() {}, "")
      };

  private GyroIO gyroIO;
  private GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

  private Rotation2d simHeading = new Rotation2d();

  private SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(Constants.Swerve.ModulePositions);
  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(
          kinematics, new Rotation2d(gyroInputs.yaw), getModulePositions(), new Pose2d());

  public Swerve(ModuleIO[] moduleIOs, GyroIO gyroIO) {

    String[] moduleNames = new String[] {"Front Left", "Front Right", "Back Left", "Back Right"};

    for (int i = 0; i < 4; i++) {
      modules[i] = new Module(moduleIOs[i], moduleNames[i]);
    }

    this.gyroIO = gyroIO;
  }

  public Command drive(Supplier<ChassisSpeeds> fieldRelativeSpeeds) {

    return Commands.run(
        () ->
            this.driveChassisSpeeds(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    fieldRelativeSpeeds.get(), getPose().getRotation())),
        this);
  }

  public void driveChassisSpeeds(ChassisSpeeds desiredSpeeds) {
    var newSpeeds = ChassisSpeeds.discretize(desiredSpeeds, 1 / Constants.mainLoopFrequency);
    var states = kinematics.toSwerveModuleStates(newSpeeds);
    for (int i = 0; i < 4; i++) {
      states[i].optimize(getModulePositions()[i].angle);
      modules[i].changeState(states[i]);
    }
    Logger.recordOutput("Swerve/ChassisSpeedSetpoint", desiredSpeeds);
    Logger.recordOutput("Swerve/Setpoints", states);
  }

  public Command resetOdometry(Pose2d newPose) {
    return Commands.runOnce(() -> poseEstimator.resetPose(newPose));
  }

  @AutoLogOutput(key = "Swerve/Positions")
  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      modules[0].getModulePosition(),
      modules[1].getModulePosition(),
      modules[2].getModulePosition(),
      modules[3].getModulePosition()
    };
  }

  @AutoLogOutput(key = "Swerve/States")
  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
      modules[0].getModuleState(),
      modules[1].getModuleState(),
      modules[2].getModuleState(),
      modules[3].getModuleState()
    };
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public void addVisionMeasurement(Pose3d visionPose, double timestamp, Matrix<N3, N1> stdDevs) {
    if (RobotBase.isReal()) {
      poseEstimator.addVisionMeasurement(getPose(), timestamp, stdDevs);
    }
  }

  public void periodic() {

    for (var module : modules) {
      module.updateInputs();
    }
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Gyro/", gyroInputs);

    if (RobotBase.isReal()) {
      poseEstimator.update(new Rotation2d(gyroInputs.yaw), getModulePositions());
    } else {
      simHeading = poseEstimator.getEstimatedPosition().getRotation();
      var gyroDelta =
          new Rotation2d(kinematics.toChassisSpeeds(getModuleStates()).omegaRadiansPerSecond);
      simHeading = simHeading.plus(gyroDelta);
      poseEstimator.update(simHeading, getModulePositions());
    }

    Logger.recordOutput("Swerve/Pose", poseEstimator.getEstimatedPosition());
  }
}
