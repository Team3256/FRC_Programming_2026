package frc.robot.subsystems.led;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.configs.LEDConfigs;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;

public final class LedConstants {
    public static final CANdleConfiguration candleConfig =
            new CANdleConfiguration().withLED(
                    new LEDConfigs().withStripType(StripTypeValue.GRB)

            );

    public static final RGBWColor orange = new RGBWColor(255, 77, 7);

    public static final RGBWColor red = new RGBWColor(255, 18, 8);

    public static final RGBWColor green = new RGBWColor(193, 255, 13);

    public static final RGBWColor purple = new RGBWColor(255, 27, 190);

    public static final int stripLength = 275;
}
