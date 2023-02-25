package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.JointMovementType;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class JointsSetPosition extends CommandBase {    
    private WristSubsystem wristSubsystem;
    private ArmSubsystem armSubsystem;
    private int armPosition;
    private int wristPosition;
    private JointMovementType movementType;
    private double waitTimeInSeconds;

    public JointsSetPosition(int armPosition, int wristPosition, JointMovementType movementType, double waitTimeInSeconds) {
        this.wristSubsystem = RobotContainer.wristSubsystem;
        this.armSubsystem = RobotContainer.armSubsystem;
        this.armPosition = armPosition;
        this.wristPosition = wristPosition;
        this.waitTimeInSeconds = waitTimeInSeconds;
        this.movementType = movementType;
    }

    @Override
    public void execute() {
        switch(movementType) {
            case MOVE_TOGETHER:
                wristSubsystem.setPosition(wristPosition)
                    .alongWith(armSubsystem.setPosition(armPosition))
                    .schedule();
                break;

            case WRIST_FIRST:
                wristSubsystem.setPosition(wristPosition)
                    .andThen(new WaitCommand(waitTimeInSeconds)
                    .andThen(armSubsystem.setPosition(armPosition)))
                    .schedule();
                break;
            
            case ARM_FIRST:
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
