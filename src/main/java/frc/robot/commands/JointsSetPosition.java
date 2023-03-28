package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class JointsSetPosition extends CommandBase {    
    private WristSubsystem wristSubsystem;
    private ArmSubsystem armSubsystem;
    private int armPosition;
    private int wristPosition;
    private int movementType;
    private double waitTimeInSeconds;

    public JointsSetPosition(int armPosition, int wristPosition, int movementType, double waitTimeInSeconds, ArmSubsystem armSubsystem, WristSubsystem wristSubsystem) {
        this.wristSubsystem = wristSubsystem;
        this.armSubsystem = armSubsystem;
        this.armPosition = armPosition;
        this.wristPosition = wristPosition;
        this.waitTimeInSeconds = waitTimeInSeconds;
        
        addRequirements(armSubsystem, wristSubsystem);
    }

    @Override
    public void execute() {
        switch(movementType) {
            case 0:
                wristSubsystem.setPosition(wristPosition)
                    .alongWith(armSubsystem.setPosition(armPosition))
                    .schedule();
                break;

            case 1:
                wristSubsystem.setPosition(wristPosition)
                    .andThen(new WaitCommand(waitTimeInSeconds)
                    .andThen(armSubsystem.setPosition(armPosition)))
                    .schedule();
                break;
            
            case 2:
                armSubsystem.setPosition(armPosition)
                    .andThen(new WaitCommand(waitTimeInSeconds)
                    .andThen(wristSubsystem.setPosition(wristPosition)))
                    .schedule();
                break;
        }
    }

    @Override
    public void end(boolean interuptted) {
        
    }
}
