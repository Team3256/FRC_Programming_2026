// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.indexer;

// TODO: FILL

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IndexerConstants {

  public static final boolean kUseFOC = false;

  public static final int kIndexerMotorID = 0; // TODO: FILL

  public static final double indexerVoltage = 0.0;

  public static double updateFrequency = 50;

  public static final TalonFXConfiguration motorConfigs =
      new TalonFXConfiguration()
          .withSlot0(new Slot0Configs().withKS(0.0).withKV(0.0).withKP(0.0).withKI(0).withKD(0))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withNeutralMode(NeutralModeValue.Brake)
                  .withInverted(InvertedValue.Clockwise_Positive))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimitEnable(true)
                  .withStatorCurrentLimit(60));

  public static int flashConfigRetries = 5;

  public static final class SimulationConstants {
    public static double rollerGearingRatio = 0.0;
    public static double rollerMomentOfInertia = 0.0;

    // Scale down the angular velocity so we can actually see what is happening
    public static double kAngularVelocityScalar = 0.0;
  }
}
