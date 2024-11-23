// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Swerve.ModuleInformation;
import frc.robot.subsystems.drivebase.GyroIO;
import frc.robot.subsystems.drivebase.GyroIO_Real;
import frc.robot.subsystems.drivebase.ModuleIO;
import frc.robot.subsystems.drivebase.ModuleIO_Real;
import frc.robot.subsystems.drivebase.ModuleIO_Sim;
import frc.robot.subsystems.drivebase.Swerve;
import frc.robot.subsystems.vision.ApriltagCamera;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {

  private CommandXboxController driver = new CommandXboxController(0);

  private Swerve drivebase;

  private ApriltagCamera camera;

  public Robot() {

    drivebase =
        new Swerve(
            RobotBase.isReal()
                ? new ModuleIO[] {
                  new ModuleIO_Real(ModuleInformation.frontLeft),
                  new ModuleIO_Real(ModuleInformation.frontRight),
                  new ModuleIO_Real(ModuleInformation.backLeft),
                  new ModuleIO_Real(ModuleInformation.backRight)
                }
                : new ModuleIO[] {
                  new ModuleIO_Sim(ModuleInformation.frontLeft),
                  new ModuleIO_Sim(ModuleInformation.frontRight),
                  new ModuleIO_Sim(ModuleInformation.backLeft),
                  new ModuleIO_Sim(ModuleInformation.backRight)
                },
            RobotBase.isReal() ? new GyroIO_Real() : new GyroIO() {});

    
  }

  @Override
  public void robotInit() {

    Logger.recordMetadata("ArborSwerveMK4i", "ArborSwerveMK4i");

    if (isReal()) {
      Logger.addDataReceiver(new WPILOGWriter());
      Logger.addDataReceiver(new NT4Publisher());
      new PowerDistribution(1, ModuleType.kRev);
    } else {
      Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
    }

    drivebase.setDefaultCommand(
        Commands.run(
            () ->
                drivebase.drive(
                    new ChassisSpeeds(
                        MathUtil.applyDeadband(-driver.getLeftY(), 0.1) * 5,
                        MathUtil.applyDeadband(-driver.getLeftX(), 0.1) * 5,
                        MathUtil.applyDeadband(-driver.getRightX(), 0.1) * 1d / 4)),
            drivebase));

    Logger.start();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }
}
