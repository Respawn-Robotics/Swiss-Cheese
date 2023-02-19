package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.WristSubsystem;

public class PickUpOffGround extends CommandBase {    
    private WristSubsystem wristSubsystem;
    private ArmSubsystem armSubsystem;

    public PickUpOffGround(WristSubsystem wristSubsystem, ArmSubsystem armSubsystem) {
        this.wristSubsystem = wristSubsystem;
        this.armSubsystem = armSubsystem;
    }

    @Override
    public void execute() {
        armSubsystem.setPosition(11000);
        wristSubsystem.setPosition(73300);
    }
}
