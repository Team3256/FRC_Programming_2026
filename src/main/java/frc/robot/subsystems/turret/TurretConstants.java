// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;

public class TurretConstants {
  public static final int turretMotorId = 21;

  public static final boolean kUseFOC = true;
  public static final boolean kUseMotionMagic = true;
  public static final int flashConfigRetries = 5;

  public static double updateFrequency = 50;
  public static final int turretEncoder1ID = 17;
  public static final int turretEncoder2ID = 0;

  public static final Rotation2d turretOffset = Rotation2d.kPi;

  public static final Transform2d driveBaseToTurret =
      new Transform2d(Units.inchesToMeters(6.750), Units.inchesToMeters(-6.750), Rotation2d.kZero);

  public static final CANcoderConfiguration cancoderConfiguration1 =
      new CANcoderConfiguration()
          .withMagnetSensor(
              new MagnetSensorConfigs()
                  .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
                  .withMagnetOffset(-0.89794921875)
                  .withAbsoluteSensorDiscontinuityPoint(Rotations.of(1)));

  public static final CANcoderConfiguration cancoderConfiguration2 =
      new CANcoderConfiguration()
          .withMagnetSensor(
              new MagnetSensorConfigs()
                  .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
                  .withMagnetOffset(-0.911376953125)
                  .withAbsoluteSensorDiscontinuityPoint(Rotations.of(1)));

  public static final TalonFXConfiguration motorConfigs =
      new TalonFXConfiguration()
          .withSlot0(
              new Slot0Configs()
                  .withKS(0)
                  .withKV(3.4812)
                  .withKP(150)
                  .withKI(0)
                  .withKD(2)
                  .withKA(0)
                  .withKG(0))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withNeutralMode(NeutralModeValue.Brake)
                  .withInverted(InvertedValue.Clockwise_Positive))
          .withMotionMagic(
              new MotionMagicConfigs()
                  .withMotionMagicAcceleration(20)
                  .withMotionMagicCruiseVelocity(12))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimitEnable(true)
                  .withStatorCurrentLimit(80))
          .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(90.0 / 10 * 48.0 / 12));

  public static final class SimulationConstants {

    public static double turretSimGearing = 1.0; // TODO: Update this value
    public static double kMomentOfInertia = 0.0001; // TODO: Update this value
    public static double kAngularVelocityScalar = 0.03;
  }

  // CRT constants

  public static final int mainTurretGear = 900;
  public static final int cancoderGear1 = 100;
  public static final int cancoderGear2 = 175;

  public static final double crtOffsetRot = 0.5;
}
