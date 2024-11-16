package frc.robot.subsystems.drivebase;

import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {

  @AutoLog
  public static class GyroIO_Inputs {
    public double yaw = 0;
    public double pitch = 0;
    public double roll = 0;
    public double yawVel = 0;
    public double xAccel = 0;
    public double yAccel = 0;
  }

  public default void updateInputs(GyroIO_Inputs inputs) {}
}
