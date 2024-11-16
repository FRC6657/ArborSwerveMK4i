// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivebase;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Swerve extends SubsystemBase {

  private Module[] modules;

  private GyroIO gyroIO;
  private GyroIO_InputsAutoLogged gyroInputs = new GyroIO_InputsAutoLogged();

  private Rotation2d simHeading = new Rotation2d();

  private SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(Constants.Swerve.ModulePositions);
  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(
          kinematics,
          new Rotation2d(gyroInputs.yaw),
          getModulePositions(),
          new Pose2d()
      );

  public Swerve(ModuleIO[] moduleIOs, GyroIO gyroIO) {
    
    modules = new Module[moduleIOs.length];

    for(int i = 0; i < modules.length; i++){
      modules[i] = new Module(moduleIOs[i], "");
    }

    this.gyroIO = gyroIO;
  }

  public SwerveModulePosition[] getModulePositions(){
    return new SwerveModulePosition[]{
      modules[0].getModulePosition(),
      modules[1].getModulePosition(),
      modules[2].getModulePosition(),
      modules[4].getModulePosition()
    };
  }

  public SwerveModuleState[] getModuleStates(){
    return new SwerveModuleState[]{
      modules[0].getModuleState(),
      modules[1].getModuleState(),
      modules[2].getModuleState(),
      modules[4].getModuleState()
    };
  }

  @Override
  public void periodic() {
    for(var module : modules){
      module.updateInputs();
    }
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Gyro/", gyroInputs);

    if (RobotBase.isReal()) {
      poseEstimator.update(new Rotation2d(gyroInputs.yaw), getModulePositions());
    } else {
      var gyroDelta = new Rotation2d(kinematics.toChassisSpeeds(getModuleStates()).omegaRadiansPerSecond * (1 / Constants.mainLoopFrequency));
      simHeading = simHeading.plus(gyroDelta);
      poseEstimator.update(simHeading, getModulePositions());
    }

  }
}
