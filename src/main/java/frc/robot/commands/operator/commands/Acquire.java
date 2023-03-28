package frc.robot.commands.operator.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.JointsSetPosition;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CollectionSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class Acquire extends CommandBase {

    private int armPosition;
    private int wristPosition;
    private boolean cubeOrCone;
    private ArmSubsystem armSubsystem;
    private WristSubsystem wristSubsystem;
    private CollectionSubsystem collectionSubsystem;
    private float secondsToWait;

    public Acquire(int armPosition, int wristPosition, boolean cubeOrCone, ArmSubsystem armSubsystem, WristSubsystem wristSubsystem, CollectionSubsystem collectionSubsystem, float secondsToWait)  {
        this.armPosition = armPosition;
        this.wristPosition = wristPosition;
        this.cubeOrCone = cubeOrCone;
        this.armSubsystem = armSubsystem;
        this.wristSubsystem = wristSubsystem;
        this.collectionSubsystem = collectionSubsystem;
        this.secondsToWait = secondsToWait;

        addRequirements(armSubsystem, wristSubsystem);
    }

    @Override
    public void execute() {
        if(cubeOrCone) {
            new JointsSetPosition(armPosition, wristPosition, 1, secondsToWait, armSubsystem, wristSubsystem)
            .alongWith(collectionSubsystem.collectCone())
            .schedule();
        } else {
            new JointsSetPosition(armPosition, wristPosition, 1, secondsToWait, armSubsystem, wristSubsystem)
            .alongWith(collectionSubsystem.collectCube())
            .schedule();
        }
    }

    @Override
    public void end(boolean interuptted) {
    }
}
