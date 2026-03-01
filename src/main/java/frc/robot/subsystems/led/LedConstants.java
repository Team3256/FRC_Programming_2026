// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.led;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.configs.LEDConfigs;
import com.ctre.phoenix6.signals.StripTypeValue;

public final class LedConstants {
  public static final CANdleConfiguration candleConfig =
      new CANdleConfiguration().withLED(new LEDConfigs().withStripType(StripTypeValue.RGB));

  public static final int stripLength = 100;
}
