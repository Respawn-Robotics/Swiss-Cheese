package frc.robot.commands.operator.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.JointMovementType;
import frc.robot.commands.JointsSetPosition;

public class Score extends CommandBase {

    private int armPosition;
    private int wristPosition;
    private boolean cubeOrCone;

    public Score(int armPosition, int wristPosition, boolean cubeOrCone)  {
        this.armPosition = armPosition;
        this.wristPosition = wristPosition;
        this.cubeOrCone = cubeOrCone;
    }

    @Override
    public void execute() {
        new JointsSetPosition(armPosition, wristPosition, JointMovementType.WRIST_FIRST, 0.4).schedule();
    }

    @Override
    public void end(boolean interuptted) {
        if(cubeOrCone) {
            RobotContainer.collectionSubsystem.ejectCone()
                .andThen(new WaitCommand(0.4)
                .andThen(new JointsSetPosition(armPosition, wristPosition, JointMovementType.WRIST_FIRST, armPosition))).schedule();
        } else {
            RobotContainer.collectionSubsystem.shootCube()
                .andThen(new WaitCommand(0.4)
                .andThen(new JointsSetPosition(armPosition, wristPosition, JointMovementType.WRIST_FIRST, armPosition))).schedule();
        }
    }
}
