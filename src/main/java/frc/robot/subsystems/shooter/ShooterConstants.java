// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public final class ShooterConstants {
  // field oriented control
  public static final boolean kUseFOC = true;
  // regen breaking

  // can IDs
  public static int shooterFollower = 60;
  public static int shooterMain = 59;

  // pid
  public static TalonFXConfiguration motorConfigs =
      new TalonFXConfiguration()
          .withSlot0(
              new Slot0Configs().withKS(0).withKV(.132).withKA(0).withKP(.4).withKI(0).withKD(0))
          // For regenerative braking
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withNeutralMode(NeutralModeValue.Coast)
                  .withInverted(InvertedValue.Clockwise_Positive))
          .withMotionMagic(new MotionMagicConfigs().withMotionMagicAcceleration(800))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimitEnable(true)
                  .withStatorCurrentLimit(60));
  public static TalonFXConfiguration followerMotorConfigs = motorConfigs;

  public static final InterpolatingDoubleTreeMap hubLUT =
      new InterpolatingDoubleTreeMap() {
        {
          put(2.0, 50.0);
          put(3.0, 50.0);
          put(4.0, 50.0);
          put(5.0, 50.0);
          put(6.0, 50.0);
        }
      };

  public static final InterpolatingDoubleTreeMap feedLUT =
      new InterpolatingDoubleTreeMap() {
        {
          put(2.0, 50.0);
          put(3.0, 50.0);
          put(4.0, 50.0);
          put(5.0, 50.0);
          put(6.0, 50.0);
        }
      };

  public static final class SimulationConstants {
    public static double kLeftGearingRatio = 1; // TODO: Update this value
    public static double kLeftMomentOfInertia = 0.01; // TODO: Update this value
    public static double kAngularVelocityScalar = .03;
  }

  // miscccc
  public static double updateFrequency = 50.0;
  public static boolean kUseMotionMagic = false;

  public static int flashConfigRetries = 5;
}
