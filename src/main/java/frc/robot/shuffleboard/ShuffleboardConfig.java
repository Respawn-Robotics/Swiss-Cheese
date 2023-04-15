package frc.robot.shuffleboard;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShuffleboardConfig {
    SendableChooser<String> autonChooser = new SendableChooser<String>();

    public ShuffleboardConfig() {
        addWidgets();
    }

    public void addWidgets() {
        autonChooser.addOption("D1 One Cone One Cube Engage", "D1ConeCubeHighE");
        autonChooser.addOption("D1 One Cone One Cube Pickup", "D1ConeCubeHighPC");
        autonChooser.addOption("D1 One Cone Leave", "D1OneCone");
        autonChooser.addOption("D2 One Cone One Cube Engage", "D2ConeCubeE");
        autonChooser.addOption("D2 One Cone One Cube", "D2ConeCube");
        autonChooser.addOption("D2 One Cone Engage", "D2ConeE");
        autonChooser.addOption("D2 One Cone Leave", "D2Cone");
        autonChooser.addOption("D2 One Cube Engage", "D2CubeE");
        autonChooser.addOption("D2 One Cube Stay", "D2Cube");
        autonChooser.addOption("D3 One Cone One Cube", "D3ConeCube");
        autonChooser.addOption("D3 One Cone", "D3OneCone");
        SmartDashboard.putData("Auton Chooser", autonChooser);
    }
}