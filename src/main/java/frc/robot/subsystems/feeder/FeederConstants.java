// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.feeder;

// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

// TODO: FILL

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class FeederConstants {

  public static final boolean kUseFOC = false;

  public static final int kFeederMotor1ID = 22;

  public static final int kFeederMotor2ID = 23;

  public static final double feederVelocity = 80;

  public static final double unjamVelocity = -40;

  public static double updateFrequency = 50;

  public static final TalonFXConfiguration motorConfigs =
      new TalonFXConfiguration()
          .withSlot0(new Slot0Configs().withKS(0.0).withKV(0.13).withKP(0.3).withKI(0).withKD(0))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withNeutralMode(NeutralModeValue.Coast)
                  .withInverted(InvertedValue.CounterClockwise_Positive))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimitEnable(true)
                  .withStatorCurrentLimit(70)
                  .withSupplyCurrentLimit(50)
                  .withSupplyCurrentLimitEnable(true)
                  .withSupplyCurrentLowerLimit(40)
                  .withSupplyCurrentLowerTime(0.1));

  public static int flashConfigRetries = 5;

  public static final class SimulationConstants {
    public static double rollerGearingRatio = 0.0;
    public static double rollerMomentOfInertia = 0.0;

    // Scale down the angular velocity so we can actually see what is happening
    public static double kAngularVelocityScalar = 0.0;
  }
}
