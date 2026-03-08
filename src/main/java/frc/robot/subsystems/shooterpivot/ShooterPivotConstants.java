// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.shooterpivot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;

public class ShooterPivotConstants {
  public static final int pivotMotorId = 36;

  public static final boolean kUseFOC = false;
  public static final boolean kUseMotionMagic = true;
  public static final int flashConfigRetries = 5;

  public static double updateFrequency = 50;

  public static final TalonFXConfiguration motorConfigs =
      new TalonFXConfiguration()
          .withSlot0(
              new Slot0Configs()
                  .withKS(0)
                  .withKV(14.22)
                  .withKP(450)
                  .withKI(0)
                  .withKD(1)
                  .withKA(0)
                  .withKG(0)
                  .withGravityType(GravityTypeValue.Arm_Cosine))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withNeutralMode(NeutralModeValue.Brake)
                  .withInverted(InvertedValue.CounterClockwise_Positive))
          .withMotionMagic(
              new MotionMagicConfigs()
                  .withMotionMagicAcceleration(8)
                  .withMotionMagicCruiseVelocity(2.5))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimitEnable(true)
                  .withStatorCurrentLimit(20))
          .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(158.142945));

  public static final InterpolatingDoubleTreeMap hubLUT =
      new InterpolatingDoubleTreeMap() {
        {
          put(0.0, 0.0);
          put(2.895, 0.0);
          put(3.33, 0.0);
          put(4.0918, 0.008545);
          put(4.785, 0.01709);
          put(5.204, 0.024658);
          put(6.096, 0.0522);
        }
      };
  public static final InterpolatingDoubleTreeMap feedLUT =
      new InterpolatingDoubleTreeMap() {
        {
          put(4.38, 0.083);
          put(6.11, 0.083);
          put(8.0, 0.083);
          put(9.42, 0.083);
          put(10.8, 0.083);
          put(12.834, 0.083);
        }
      };

  public static final class PivotSim {
    public static final double pivotSimGearing = 40;

    public static final Distance shooterPivotLength = Inches.of(12);
    public static final Mass shooterPivotMass = Kilograms.of(1);
    public static final double jkGMetersSquared = 1;

    public static final Rotation2d minAngle = Rotation2d.fromDegrees(0.0);
    public static final Rotation2d maxAngle = Rotation2d.fromDegrees(45);
    public static final Rotation2d startingAngle = Rotation2d.fromDegrees(0);
  }
}
