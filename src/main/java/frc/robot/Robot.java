// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoFactory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.swerve.GyroIO;
import frc.robot.subsystems.swerve.GyroIO_Real;
import frc.robot.subsystems.swerve.ModuleIO;
import frc.robot.subsystems.swerve.ModuleIO_Real;
import frc.robot.subsystems.swerve.ModuleIO_Sim;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.vision.ApriltagCameraIO_Real;
import frc.robot.subsystems.vision.ApriltagCameraIO_Sim;
import frc.robot.subsystems.vision.ApriltagCameras;
import frc.robot.subsystems.vision.VisionConstants;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {

  private CommandXboxController driver = new CommandXboxController(0);

  private final Swerve swerve;
  private final ApriltagCameras cameras;

  private Superstructure superstructure;

  private final AutoFactory autoFactory;

  public Robot() {

    swerve =
        new Swerve(
            RobotBase.isReal() ? new GyroIO_Real() : new GyroIO() {},
            new ModuleIO[] {
              RobotBase.isReal()
                  ? new ModuleIO_Real(SwerveConstants.frontLeftModule)
                  : new ModuleIO_Sim(SwerveConstants.frontLeftModule),
              RobotBase.isReal()
                  ? new ModuleIO_Real(SwerveConstants.frontRightModule)
                  : new ModuleIO_Sim(SwerveConstants.frontRightModule),
              RobotBase.isReal()
                  ? new ModuleIO_Real(SwerveConstants.backLeftModule)
                  : new ModuleIO_Sim(SwerveConstants.backLeftModule),
              RobotBase.isReal()
                  ? new ModuleIO_Real(SwerveConstants.backRightModule)
                  : new ModuleIO_Sim(SwerveConstants.backRightModule)
            });

    cameras =
        new ApriltagCameras(
            swerve::addVisionMeasurement,
            RobotBase.isReal() || replay
                ? new ApriltagCameraIO_Real(VisionConstants.cameraInfo)
                : new ApriltagCameraIO_Sim(VisionConstants.cameraInfo, swerve::getPose),
            RobotBase.isReal() || replay
                ? new ApriltagCameraIO_Real(VisionConstants.cameraInfo)
                : new ApriltagCameraIO_Sim(VisionConstants.cameraInfo, swerve::getPose));

    superstructure = new Superstructure(swerve);

    autoFactory =
        new AutoFactory(swerve::getPose, swerve::resetPose, swerve::followTrajectory, true, swerve);
  }

  public static boolean replay = false;

  @SuppressWarnings("resource")
  @Override
  public void robotInit() {

    swerve.setDefaultCommand(
        swerve.driveTeleop(
            () ->
                new ChassisSpeeds(
                    -MathUtil.applyDeadband(driver.getLeftY(), 0.1)
                        * SwerveConstants.maxLinearSpeed
                        * 0.3,
                    -MathUtil.applyDeadband(driver.getLeftX(), 0.1)
                        * SwerveConstants.maxLinearSpeed
                        * 0.3,
                    -MathUtil.applyDeadband(driver.getRightX(), 0.1)
                        * SwerveConstants.maxAngularSpeed
                        * 0.3)));

    Logger.recordMetadata("ArborSwerveMK4i", "ArborSwerveMK4i");

    if (isReal()) {
      Logger.addDataReceiver(new WPILOGWriter());
      Logger.addDataReceiver(new NT4Publisher());
      new PowerDistribution(1, ModuleType.kRev);
    } else {
      if (!replay) {
        Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
      } else {
        setUseTiming(false);
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_replay")));
      }
    }

    Logger.start();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    superstructure.testAuto(autoFactory).cmd().schedule();
  }
}
