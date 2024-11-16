package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class Constants {

  public static int mainLoopFrequency = 50; // Hz

  public static enum CAN {
    Swerve_FL_D(1),
    Swerve_FR_D(2),
    Swerve_BL_D(3),
    Swerve_BR_D(4),
    Swerve_FL_T(5),
    Swerve_FR_T(6),
    Swerve_BL_T(7),
    Swerve_BR_T(8),
    Gyro(9);

    public int id;

    CAN(int id) {
      this.id = id;
    }
  }

  public static class Motors {

    public static double KrakenRPS =
        Units.radiansPerSecondToRotationsPerMinute(DCMotor.getKrakenX60(1).freeSpeedRadPerSec) / 60;
    public static double FalconRPS =
        Units.radiansPerSecondToRotationsPerMinute(DCMotor.getFalcon500(1).freeSpeedRadPerSec) / 60;
  }

  public static class Swerve {

    public static double WheelDiameter = Units.inchesToMeters(4);
    public static double TrackWidth = 26 - 5.25; // Inches
    public static double TrackLength = 26 - 5.25; // Inches

    public static Translation2d[] ModulePositions =
        new Translation2d[] {
          new Translation2d(TrackWidth / 2, TrackLength / 2),
          new Translation2d(-TrackWidth / 2, TrackLength / 2),
          new Translation2d(-TrackWidth / 2, -TrackLength / 2),
          new Translation2d(TrackWidth / 2, -TrackLength / 2)
        };

    public static double TurnGearing = 150d / 7d;

    public enum DriveGearing {
      L1(19d / 25d),
      L2(17d / 27d),
      L3(16d / 28d);

      public double reduction;

      DriveGearing(double reduction) {
        this.reduction = reduction * (50d / 14d) * (45d / 15d);
      }
    }

    public static class ModuleInformation {

      public String name;
      public int driveID;
      public int turnID;
      public int encoderID;
      public double encoderOffset;

      public ModuleInformation(
          String name, int driveID, int turnID, int encoderID, double encoderOffset) {
        this.name = name;
        this.driveID = driveID;
        this.turnID = turnID;
        this.encoderID = encoderID;
        this.encoderOffset = encoderOffset;
      }
    }
  }
}
