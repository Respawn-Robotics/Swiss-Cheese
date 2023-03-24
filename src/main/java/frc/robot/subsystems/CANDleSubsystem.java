package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;

public class CANDleSubsystem {
    CANdle candle = new CANdle(17);

    public CANDleSubsystem() {
        CANdleConfiguration config = new CANdleConfiguration();
        config.stripType = LEDStripType.RGB; // set the strip type to RGB
        config.brightnessScalar = 1;
        candle.configAllSettings(config);

        RainbowAnimation anim = new RainbowAnimation(1, 1, 1000);
        candle.animate(anim);
    }
}
