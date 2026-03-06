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

    public static final RGBWColor orange = new RGBWColor(255, 77, 7, 50);

    public static final RGBWColor disabledRed = new RGBWColor(255, 0, 0, 50);

    public static final int stripLength = 500;
}
