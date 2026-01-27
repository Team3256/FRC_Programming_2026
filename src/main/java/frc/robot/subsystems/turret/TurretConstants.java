// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.turret;

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
import edu.wpi.first.math.geometry.Transform2d;

public class TurretConstants {
  public static final int turretMotorId = 30;

  public static final boolean kUseFOC = false;
  public static final boolean kUseMotionMagic = true;
  public static final int flashConfigRetries = 5;

  public static double updateFrequency = 50;

  public static final Rotation2d turretOffset = new Rotation2d();

  public static final Transform2d turretToDrivebase = new Transform2d();

  public static final TalonFXConfiguration motorConfigs =
      new TalonFXConfiguration()
          .withSlot0(
              new Slot0Configs()
                  .withKS(0)
                  .withKV(0)
                  .withKP(0)
                  .withKI(0)
                  .withKD(0)
                  .withKA(0)
                  .withKG(0)
                  .withGravityType(GravityTypeValue.Arm_Cosine))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withNeutralMode(NeutralModeValue.Brake)
                  .withInverted(InvertedValue.CounterClockwise_Positive))
          .withMotionMagic(
              new MotionMagicConfigs()
                  .withMotionMagicAcceleration(10)
                  .withMotionMagicCruiseVelocity(2))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimitEnable(true)
                  .withStatorCurrentLimit(80))
          .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(27.32));

  public static final class SimulationConstants {

    public static double turretSimGearing = 1.0; // TODO: Update this value
    public static double kMomentOfInertia = 0.01; // TODO: Update this value
    public static double kAngularVelocityScalar = 0.03;
  }
}
