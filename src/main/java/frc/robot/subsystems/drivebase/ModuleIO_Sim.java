package frc.robot.subsystems.drivebase;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.Constants.Motors;
import frc.robot.Constants.Swerve;
import frc.robot.Constants.Swerve.ModuleInformation;

public class ModuleIO_Sim implements ModuleIO {

  private TalonFX drive;
  private TalonFX turn;

  private DCMotorSim driveSim;
  private DCMotorSim turnSim;

  private VelocityVoltage driveControl;
  private PositionVoltage turnControl;

  public ModuleIO_Sim(ModuleInformation moduleInformation) {

    drive = new TalonFX(moduleInformation.driveID);
    turn = new TalonFX(moduleInformation.turnID);

    var driveConfig = new TalonFXConfiguration();
    driveConfig.Feedback.SensorToMechanismRatio = Swerve.DriveGearing.L3.reduction;
    driveConfig.Slot0.kS = 0;
    driveConfig.Slot0.kA = 0;
    driveConfig.Slot0.kV = (12d / (Motors.KrakenRPS * Swerve.DriveGearing.L3.reduction));
    driveConfig.Slot0.kP = 0;
    driveConfig.Slot0.kD = 0;

    var turnConfig = new TalonFXConfiguration();
    turnConfig.Feedback.SensorToMechanismRatio = Swerve.TurnGearing;
    turnConfig.Slot0.kS = 0;
    turnConfig.Slot0.kA = 0;
    turnConfig.Slot0.kV = (12d / (Motors.FalconRPS * Swerve.TurnGearing));
    turnConfig.Slot0.kP = 0;
    turnConfig.Slot0.kD = 0;
    turnConfig.ClosedLoopGeneral.ContinuousWrap = true;

    driveSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60(1), 0.025, Swerve.DriveGearing.L3.reduction),
            DCMotor.getKrakenX60(1));

    turnSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getFalcon500(1), 0.004, Swerve.TurnGearing),
            DCMotor.getFalcon500(1));
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {

    var driveSimState = drive.getSimState();
    var turnSimState = turn.getSimState();

    driveSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    turnSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

    var driveApplied = driveSimState.getMotorVoltage();
    var turnApplied = driveSimState.getMotorVoltage();

    driveSim.setInputVoltage(driveApplied);
    turnSim.setInputVoltage(turnApplied);

    driveSim.update(1 / Constants.mainLoopFrequency);
    turnSim.update(1 / Constants.mainLoopFrequency);

    driveSimState.setRawRotorPosition(
        driveSim.getAngularPosition().times(Swerve.DriveGearing.L3.reduction));
    driveSimState.setRotorVelocity(
        turnSim.getAngularVelocity().times(Swerve.DriveGearing.L3.reduction));

    turnSimState.setRawRotorPosition(turnSim.getAngularPosition().times(Swerve.TurnGearing));
    turnSimState.setRotorVelocity(turnSim.getAngularVelocity().times(Swerve.TurnGearing));

    inputs.driveApplied = driveApplied;
    inputs.driveSupplyCurrent = driveSim.getCurrentDrawAmps();
    inputs.drivePosition = drive.getPosition().getValueAsDouble() * Swerve.WheelDiameter * Math.PI;
    inputs.driveVelocity = drive.getVelocity().getValueAsDouble() * Swerve.WheelDiameter * Math.PI;
    inputs.driveAcceleration =
        drive.getAcceleration().getValueAsDouble() * Swerve.WheelDiameter * Math.PI;

    inputs.turnApplied = turnApplied;
    inputs.turnSupplyCurrent = turnSim.getCurrentDrawAmps();
    inputs.turnPosition = turn.getPosition().getValueAsDouble() * 2 * Math.PI;
    inputs.turnVelocity = turn.getVelocity().getValueAsDouble() * 2 * Math.PI;
    inputs.turnAcceleration = turn.getAcceleration().getValueAsDouble() * 2 * Math.PI;

    inputs.encoderAbsPosition = turn.getPosition().getValueAsDouble() * 2 * Math.PI;
    inputs.encoderRelPosition = turn.getPosition().getValueAsDouble() * 2 * Math.PI;
    inputs.encoderVelocity = turn.getVelocity().getValueAsDouble() * 2 * Math.PI;
  }

  @Override
  public void changeDriveSetpoint(double mps) {
    drive.setControl(driveControl.withSlot(0).withVelocity(mps / (Swerve.WheelDiameter * Math.PI)));
  }

  @Override
  public void changeTurnSetpoint(double rad) {
    turn.setControl(turnControl.withSlot(0).withPosition(rad / (2 * Math.PI)));
  }

  @Override
  public SwerveModulePosition getModulePosition() {
    return new SwerveModulePosition(drive.getPosition().getValueAsDouble() * Swerve.WheelDiameter * Math.PI, new Rotation2d(turn.getPosition().getValueAsDouble() * 2 * Math.PI));
  }

  @Override
  public SwerveModuleState getModuleState() {
    return new SwerveModuleState(drive.getVelocity().getValueAsDouble() * Swerve.WheelDiameter * Math.PI, new Rotation2d(turn.getVelocity().getValueAsDouble() * 2 * Math.PI));
  }

}
