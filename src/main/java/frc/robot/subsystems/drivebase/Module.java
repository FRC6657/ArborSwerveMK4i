package frc.robot.subsystems.drivebase;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.Logger;

public class Module {

  private ModuleIO io;
  private ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private String name;

  public Module(ModuleIO io, String name) {
    this.io = io;
    this.name = name;
  }

  public SwerveModulePosition getModulePosition() {
    return io.getModulePosition();
  }

  public SwerveModuleState getModuleState() {
    return io.getModuleState();
  }

  public void changeState(SwerveModuleState desiredState) {
    io.changeDriveSetpoint(desiredState.speedMetersPerSecond);
    io.changeTurnSetpoint(desiredState.angle.getRadians());
  }

  public void updateInputs() {
    io.updateInputs(inputs);
    Logger.processInputs("Swerve/" + name + "Module", inputs);
  }
}
