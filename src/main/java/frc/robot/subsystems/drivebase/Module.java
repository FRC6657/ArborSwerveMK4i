package frc.robot.subsystems.drivebase;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.Logger;

public class Module {

  // Module IOs
  private ModuleIO io;
  private ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();

  // Name to track the module's name
  private String name;

  public Module(ModuleIO io, String name) {
    this.io = io;
    this.name = name;
  }

  // Get the Swerve Module's Position
  public SwerveModulePosition getModulePosition() {
    return io.getModulePosition();
  }

  // Get the Swerve Module's State
  public SwerveModuleState getModuleState() {
    return io.getModuleState();
  }

  // Set the new desired Module Setpoint
  public void changeState(SwerveModuleState desiredState) {
    io.changeDriveSetpoint(desiredState.speedMetersPerSecond);
    io.changeTurnSetpoint(desiredState.angle.getRadians());
  }

  // Update Module IO
  public void updateInputs() {
    io.updateInputs(inputs);
    Logger.processInputs("Swerve/" + name + " Module", inputs);
  }
}
