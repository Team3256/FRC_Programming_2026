package frc.robot.subsystems.led;

import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.PhoenixUtil;

public class Led extends SubsystemBase {

    private CANdle candle = new CANdle(0);
    public Led() {
        super();

        candle.getConfigurator().apply(LedConstants.candleConfig);
    }


    private Command setStrobe(RGBWColor color, double frameRate) {
        return this.runOnce(()->candle.setControl(new StrobeAnimation(0, LedConstants.stripLength).withColor(color).withFrameRate(frameRate))).ignoringDisable(true);
    }
    private Command setColorCmd(RGBWColor color) {
        return this.runOnce(()->this.setColor(color));
    }

    private void setColor(RGBWColor color) {
        candle.setControl(new EmptyAnimation(0));
        candle.setControl(new SolidColor(0, LedConstants.stripLength).withColor(color));
    }

    private Command halfStrobeHalfColor(RGBWColor color1, RGBWColor color2, double frameRate   ) {
        return this.runOnce(()->{
           candle.setControl(new EmptyAnimation(0));
           candle.setControl(new StrobeAnimation(0, LedConstants.stripLength/2).withColor(color1).withFrameRate(frameRate));
           candle.setControl(new SolidColor(LedConstants.stripLength/2+1, LedConstants.stripLength).withColor(color2));
        });


    }

    public void setPurple() {
        setColor(LedConstants.purple);
    }

    public Command setRed() {
        return setColorCmd(LedConstants.red);
    }

    public Command strobeOrange() {
        return setStrobe(LedConstants.orange, 4);
    }

    public Command setGreen() {
        return setColorCmd(LedConstants.green);
    }

    public Command halfOrangeHalfGreen() {
        return halfStrobeHalfColor(LedConstants.orange, LedConstants.green, 4);
    }


}
