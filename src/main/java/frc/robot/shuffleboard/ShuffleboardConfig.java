package frc.robot.shuffleboard;

import java.util.Map;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.autos.AutonChooser;

public class ShuffleboardConfig {

    public ShuffleboardConfig() {
        addWidgets();
    }
    
    public void addWidgets() {
        Shuffleboard.getTab("Swiss Cheese")
            .add("Auton", "Gyro")
            .withWidget(BuiltInWidgets.kComboBoxChooser)
            .withProperties(Map.of("Swerve S", "Gyro"))
            .getEntry();
            Shuffleboard.getTab("Swiss Cheese")
            .add(AutonChooser.chooser);
    }
}
