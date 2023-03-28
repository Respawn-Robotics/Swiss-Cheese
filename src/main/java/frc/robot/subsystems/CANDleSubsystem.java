package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CANDleSubsystem extends SubsystemBase {
    CANdle candle = new CANdle(17);

    public CANDleSubsystem() {
        CANdleConfiguration config = new CANdleConfiguration();
        config.stripType = LEDStripType.RGB; // set the strip type to RGB
        config.brightnessScalar = 1;
        candle.configAllSettings(config);
    }

    public void changeColor(int r, int g, int b) {
        candle.animate(new StrobeAnimation(r, b, g, 0, 1, 1000, 8));
    }

    public Command setYellow() {
        return runOnce(() -> {
            changeColor(255, 160, 0);
        });
    }

    public Command setPurple() {
        return runOnce(() -> {
            changeColor(255, 32, 255);
        });
    }

    public Command setRed() {
        return runOnce(() -> {
            changeColor(255, 0, 0);
        });
    }

    public Command setGreen() {
        return runOnce(() -> {
            changeColor(0, 255, 0);
        });
    }

    public Command setBlue() {
        return runOnce(() -> {
            changeColor(0, 0, 255);
        });
    }
}
