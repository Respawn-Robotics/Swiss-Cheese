package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ManualArmUp extends CommandBase {    
    private ArmSubsystem armSubsystem;

    public ManualArmUp(ArmSubsystem armSubsystem) {
        this.armSubsystem = armSubsystem;
    }

    @Override
    public void execute() {
        armSubsystem.setVoltage(0.1f).schedule();
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.setVoltage(0).schedule();
    }
}