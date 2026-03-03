// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.led;

import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Led extends SubsystemBase {

  private CANdle candle = new CANdle(0);

  public Led() {
    super();

    candle.getConfigurator().apply(LedConstants.candleConfig);
  }

  private Command setStrobe(RGBWColor color, double frameRate) {
    return this.runOnce(
            () ->
                candle.setControl(
                    new StrobeAnimation(0, LedConstants.stripLength)
                        .withColor(color)
                        .withFrameRate(frameRate)))
        .ignoringDisable(true);
  }

  public Command setRed() {
    return setStrobe(RGBWColor.fromHSV(0, 0, 255), 20);
  }

  public Command setGreen() {
    return setStrobe(RGBWColor.fromHSV(120, 255, 255), 20);
  }
}
