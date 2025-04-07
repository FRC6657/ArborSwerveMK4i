package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class SwerveConstants {

  public static enum CAN {
    Swerve_FR_D(1),
    Swerve_FL_D(2),
    Swerve_BR_D(3),
    Swerve_BL_D(4),
    Swerve_FR_T(5),
    Swerve_FL_T(6),
    Swerve_BR_T(7),
    Swerve_BL_T(8),
    Swerve_FR_E(9),
    Swerve_FL_E(10),
    Swerve_BR_E(11),
    Swerve_BL_E(12),
    Gyro(13);

    public final int id;

    private CAN(int id) {
      this.id = id;
    }
  }

  public record ModuleConstants(int id, String prefix, int driveID, int turnID, int encoderID) {}

  public static final double trackWidthX = Units.inchesToMeters(23.75);
  public static final double trackWidthY = Units.inchesToMeters(23.75);

  public static final double maxLinearSpeed = Units.feetToMeters(17.3);
  public static final double maxLinearAcceleration = 8;
  public static final double maxAngularSpeed =
      maxLinearSpeed / Math.hypot(trackWidthX / 2.0, trackWidthY / 2.0);

  public static final ModuleConstants frontLeftModule =
      new ModuleConstants(
          0, "Front Left", CAN.Swerve_FL_D.id, CAN.Swerve_FL_T.id, CAN.Swerve_FL_E.id);
  public static final ModuleConstants frontRightModule =
      new ModuleConstants(
          0, "Front Right", CAN.Swerve_FR_D.id, CAN.Swerve_FR_T.id, CAN.Swerve_FR_E.id);
  public static final ModuleConstants backLeftModule =
      new ModuleConstants(
          0, "Back Left", CAN.Swerve_BL_D.id, CAN.Swerve_BL_T.id, CAN.Swerve_BL_E.id);
  public static final ModuleConstants backRightModule =
      new ModuleConstants(
          0, "Back Right", CAN.Swerve_BR_D.id, CAN.Swerve_BR_T.id, CAN.Swerve_BR_E.id);

  public static final double wheelRadiusMeters = Units.inchesToMeters(2);
  public static final double driveRatio = (45.0 / 15.0) * (16.0 / 28.0) * (50.0 / 14.0);
  public static final double turnRatio = (150.0 / 7.0);
  public static final double driveRotorToMeters = driveRatio / (wheelRadiusMeters * 2 * Math.PI);

  public static final Translation2d[] moduleTranslations = {
    new Translation2d(trackWidthX / 2, trackWidthY / 2),
    new Translation2d(trackWidthX / 2, -trackWidthY / 2),
    new Translation2d(-trackWidthX / 2, trackWidthY / 2),
    new Translation2d(-trackWidthX / 2, -trackWidthY / 2)
  };

  public static TalonFXConfiguration driveConfig =
      new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withSupplyCurrentLimit(40)
                  .withSupplyCurrentLimitEnable(true)
                  .withStatorCurrentLimit(80)
                  .withStatorCurrentLimitEnable(true))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(InvertedValue.Clockwise_Positive)
                  .withNeutralMode(NeutralModeValue.Brake))
          .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(driveRotorToMeters))
          .withSlot0(new Slot0Configs().withKV(12d / maxLinearSpeed).withKS(0).withKP(2.25));

  public static TalonFXConfiguration turnConfig =
      new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withSupplyCurrentLimit(20)
                  .withSupplyCurrentLimitEnable(true)
                  .withStatorCurrentLimit(40)
                  .withStatorCurrentLimitEnable(true))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(InvertedValue.Clockwise_Positive)
                  .withNeutralMode(NeutralModeValue.Brake))
          .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(turnRatio))
          .withSlot0(
              new Slot0Configs()
                  .withKV(12d / ((6300d / 60) / turnRatio))
                  .withKS(0.27)
                  .withKP(25)
                  .withKD(0.7))
          .withMotionMagic(
              new MotionMagicConfigs()
                  .withMotionMagicCruiseVelocity((6300 / 60) / turnRatio)
                  .withMotionMagicAcceleration((6300 / 60) / (turnRatio * 0.001)))
          .withClosedLoopGeneral(new ClosedLoopGeneralConfigs().withContinuousWrap(true));
}
