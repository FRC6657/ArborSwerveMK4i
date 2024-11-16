package frc.robot.subsystems.drivebase;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {

  @AutoLog
  public static class ModuleIOInputs {
    public double driveApplied = 0; // Volts
    public double driveStatorCurrent = 0; // Amps
    public double driveSupplyCurrent = 0; // Amps
    public double drivePosition = 0; // Meters
    public double driveVelocity = 0; // Meters/s
    public double driveAcceleration = 0; // Meters/s/s
    public double driveTemp = 0; // Celcius

    public double turnApplied = 0; // Volts
    public double turnStatorCurrent = 0; // Amps
    public double turnSupplyCurrent = 0; // Amps
    public double turnPosition = 0; // Rad
    public double turnVelocity = 0; // Rad/s
    public double turnAcceleration = 0; // Rad/s/s
    public double turnTemp = 0; // Celcius

    public double encoderAbsPosition = 0; // Rad
    public double encoderRelPosition = 0; // Rad
    public double encoderVelocity = 0; // Rad/s
  }

  public default void updateInputs(ModuleIOInputs inputs) {}

  public default void changeDriveSetpoint(double mps) {}

  public default void changeTurnSetpoint(double rad) {}

  public default SwerveModulePosition getModulePosition() {
    return new SwerveModulePosition();
  }

  public default SwerveModuleState getModuleState() {
    return new SwerveModuleState();
  }
}
