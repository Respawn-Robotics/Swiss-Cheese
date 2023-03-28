package frc.robot.commands.manual;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WristSubsystem;

public class ManualWristDown extends CommandBase {    
    private WristSubsystem wristSubsystem;

    public ManualWristDown(WristSubsystem wristSubsystem) {
        this.wristSubsystem = wristSubsystem;
    }

    @Override
    public void execute() {
        wristSubsystem.setVoltage(-0.2f).schedule();
    }

    @Override
    public void end(boolean interrupted) {
        wristSubsystem.setVoltage(0f).schedule();
    }
}
