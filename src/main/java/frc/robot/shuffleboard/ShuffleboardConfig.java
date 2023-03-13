package frc.robot.shuffleboard;

import java.util.Map;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.autos.autoCommands.D1OneCone;

public class ShuffleboardConfig {
    SendableChooser<String> autonChooser = new SendableChooser<String>();

    public ShuffleboardConfig() {
        addWidgets();
    }
   
    public void addWidgets() {
        autonChooser.addOption("D1 One Cone Leave", "D1OneCone");
        autonChooser.addOption("D1 Two Cube Engage", "D1TwoCube");
        autonChooser.addOption("D2 Score Cube Pick Up Cube Engage", "D2CubeEngage");
        autonChooser.addOption("D2 One Cone Engage", "D2OneCone");
        autonChooser.addOption("D3 One Cone Leave", "D3OneCone");
        SmartDashboard.putData("Auton Chooser", autonChooser);
    }
}