package frc.robot.commands.manual;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ManualArmDown extends CommandBase {    
    private ArmSubsystem armSubsystem;

    public ManualArmDown(ArmSubsystem armSubsystem) {
        this.armSubsystem = armSubsystem;
    }

    @Override
    public void execute() {
        armSubsystem.setVoltage(-0.2f).schedule();
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.setVoltage(0f).schedule();
    }
}
