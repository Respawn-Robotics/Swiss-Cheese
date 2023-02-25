package frc.robot.commands.operator.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.JointMovementType;
import frc.robot.commands.JointsSetPosition;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CollectionSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class Score extends CommandBase {

    private int armPosition;
    private int wristPosition;
    private boolean cubeOrCone;
    private ArmSubsystem armSubsystem;
    private WristSubsystem wristSubsystem;
    private CollectionSubsystem collectionSubsystem;

    public Score(int armPosition, int wristPosition, boolean cubeOrCone, ArmSubsystem armSubsystem, WristSubsystem wristSubsystem, CollectionSubsystem collectionSubsystem)  {
        this.armPosition = armPosition;
        this.wristPosition = wristPosition;
        this.cubeOrCone = cubeOrCone;
        this.armSubsystem = armSubsystem;
        this.wristSubsystem = wristSubsystem;
        this.collectionSubsystem = collectionSubsystem;

        addRequirements(armSubsystem, wristSubsystem);
    }

    @Override
    public void execute() {
        new JointsSetPosition(armPosition, wristPosition, 1, 0.4, armSubsystem, wristSubsystem).schedule();
    }

    @Override
    public void end(boolean interuptted) {
        if(cubeOrCone) {
            collectionSubsystem.ejectCone()
                .andThen(new WaitCommand(0.4)
                .andThen(new JointsSetPosition(armPosition, wristPosition, 1, armPosition, armSubsystem, wristSubsystem))).schedule();
        } else {
            collectionSubsystem.shootCube()
                .andThen(new WaitCommand(0.4)
                .andThen(new JointsSetPosition(armPosition, wristPosition, 1, armPosition, armSubsystem, wristSubsystem))).schedule();
        }
    }
}
