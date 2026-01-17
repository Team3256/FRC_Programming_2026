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
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public final class ShooterConstants {
  // field oriented control
  public static final boolean kUseFOC = true;
  // regen breaking
  public static final boolean kUseShooterRegenBraking = true;

  // can IDs
  public static int shooterFollower = 0;
  public static int shooterMain = 1;

  // motor output behavior
  public static MotorOutputConfigs motorOutputConfigs =
      new MotorOutputConfigs()
          .withNeutralMode(NeutralModeValue.Brake)
          .withInverted(InvertedValue.CounterClockwise_Positive);

  // pid
  public static TalonFXConfiguration motorConfigs =
      new TalonFXConfiguration()
          .withSlot0(new Slot0Configs().withKS(0).withKV(0).withKA(0).withKP(8).withKI(0).withKD(0))
          // For regenerative braking
          // we need to make sure that the backcurrent is below the breaker limit
          // P = 2 gives us like 102 amps so that's good enough

          .withSlot1(new Slot1Configs().withKS(0).withKV(0).withKP(2).withKI(0).withKD(0))
          .withMotorOutput(motorOutputConfigs)
          .withMotionMagic(
              new MotionMagicConfigs()
                  .withMotionMagicAcceleration(1600)
                  .withMotionMagicCruiseVelocity(0))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimitEnable(true)
                  .withStatorCurrentLimit(60))
          .withTorqueCurrent(
              new TorqueCurrentConfigs()
                  .withPeakForwardTorqueCurrent(80)
                  .withPeakReverseTorqueCurrent(80));
  public static TalonFXConfiguration followerMotorConfigs =
      motorConfigs;

  public static final class SimulationConstants {
    public static double kLeftGearingRatio = 1.0; // TODO: Update this value
    public static double kLeftMomentOfInertia = 0.0001; // TODO: Update this value
    public static double kAngularVelocityScalar = 0.05;
  }

  // miscccc
  public static double updateFrequency = 50.0;
  public static boolean kUseMotionMagic = false;

  public static int flashConfigRetries = 5;
}
