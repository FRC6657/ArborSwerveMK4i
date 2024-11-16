package frc.robot.subsystems.drivebase;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import frc.robot.Constants;
import frc.robot.Constants.CAN;

public class GyroIO_Real implements GyroIO {

  public Pigeon2 gyro;

  StatusSignal<Angle> yaw;
  StatusSignal<Angle> pitch;
  StatusSignal<Angle> roll;
  StatusSignal<AngularVelocity> yawVel;
  StatusSignal<LinearAcceleration> xAccel;
  StatusSignal<LinearAcceleration> yAccel;

  public GyroIO_Real() {

    gyro = new Pigeon2(CAN.Gyro.id);

    yaw = gyro.getYaw();
    pitch = gyro.getPitch();
    roll = gyro.getRoll();
    yawVel = gyro.getAngularVelocityZDevice();
    xAccel = gyro.getAccelerationX();
    yAccel = gyro.getAccelerationY();

    yaw.setUpdateFrequency(Constants.mainLoopFrequency);
    pitch.setUpdateFrequency(Constants.mainLoopFrequency);
    roll.setUpdateFrequency(Constants.mainLoopFrequency);
    yawVel.setUpdateFrequency(Constants.mainLoopFrequency);
    xAccel.setUpdateFrequency(Constants.mainLoopFrequency);
    yAccel.setUpdateFrequency(Constants.mainLoopFrequency);

    gyro.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(GyroIO_Inputs inputs) {
    inputs.yaw = Units.degreesToRadians(yaw.getValueAsDouble());
    inputs.pitch = Units.degreesToRadians(pitch.getValueAsDouble());
    inputs.roll = Units.degreesToRadians(roll.getValueAsDouble());
    inputs.yawVel = Units.degreesToRadians(yawVel.getValueAsDouble());
    inputs.xAccel = xAccel.getValueAsDouble() * 9.81;
    inputs.yAccel = yAccel.getValueAsDouble() * 9.81;
  }
}