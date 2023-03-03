package frc.robot.commands.operator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.JointMovementType;
import frc.robot.Constants.SuperstructureConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.JointsSetPosition;
import frc.robot.commands.operator.commands.Acquire;
import frc.robot.commands.operator.commands.Score;
import frc.robot.drivers.BeamBreak;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CollectionSubsystem;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.WristSubsystem;

public class OperatorCommands {

    private ArmSubsystem armSubsystem;
    private WristSubsystem wristSubsystem;
    private CollectionSubsystem collectionSubsystem;

    public OperatorCommands(ArmSubsystem armSubsystem, WristSubsystem wristSubsystem, CollectionSubsystem collectionSubsystem) {
        this.armSubsystem = armSubsystem;
        this.wristSubsystem = wristSubsystem;
        this.collectionSubsystem = collectionSubsystem;
    }

    public Command goToHome() {
        return new JointsSetPosition(0, 0, 1, 2, armSubsystem, wristSubsystem);
        //return armSubsystem.setPosition(8000).andThen(new WaitCommand(0.6).andThen(new JointsSetPosition(0, 0, 1, 0.4, armSubsystem, wristSubsystem)));
    }
    
    public Command acquireConeFromFloor() {
        Superstructure.currentRobotState = SuperstructureConstants.ROBOT_STATE.ACQUIRE_CONE_FLOOR;
        return new Acquire(ArmConstants.ACQUIRE_FROM_CONE_FLOOR, WristConstants.ACQUIRE_FROM_CONE_FLOOR, true, armSubsystem, wristSubsystem, collectionSubsystem, 0.4f);
    }

    public Command acquireCubeFromFloor() {
        Superstructure.currentRobotState = SuperstructureConstants.ROBOT_STATE.ACQUIRE_CUBE_FLOOR;
        return new Acquire(ArmConstants.ACQUIRE_FROM_CUBE_FLOOR, WristConstants.ACQUIRE_FROM_CUBE_FLOOR, false, armSubsystem, wristSubsystem, collectionSubsystem, 0.4f);
    }

    public Command acquireConeFromDoS() {
        Superstructure.currentRobotState = SuperstructureConstants.ROBOT_STATE.ACQUIRE_CONE_DOS;
        return new Acquire(ArmConstants.ACQUIRE_FROM_DOS, WristConstants.ACQUIRE_FROM_DOS, false, armSubsystem, wristSubsystem, collectionSubsystem, 2f);
    }

    public Command acquireCubeFromDoS() {
        Superstructure.currentRobotState = SuperstructureConstants.ROBOT_STATE.ACQUIRE_CUBE_DOS;
        return new Acquire(ArmConstants.ACQUIRE_FROM_DOS, WristConstants.ACQUIRE_FROM_DOS, true, armSubsystem, wristSubsystem, collectionSubsystem, 2f);
    }

    public Command scoreInHighCone() {
        return new Score(ArmConstants.SCORE_IN_HIGH_CONE, WristConstants.SCORE_IN_HIGH_CONE, false, armSubsystem, wristSubsystem, collectionSubsystem);
    }

    public Command scoreInHighCube() {
        return new Score(ArmConstants.SCORE_IN_HIGH_CUBE, WristConstants.SCORE_IN_HIGH_CUBE, true, armSubsystem, wristSubsystem, collectionSubsystem);
    }

    public Command scoreInMidCone() {
        return new Score(ArmConstants.SCORE_IN_MID_CONE, WristConstants.SCORE_IN_MID_CONE, false, armSubsystem, wristSubsystem, collectionSubsystem);
    }

    public Command scoreInMidCube() {
        return new Score(ArmConstants.SCORE_IN_MID_CUBE, WristConstants.SCORE_IN_MID_CUBE, true, armSubsystem, wristSubsystem, collectionSubsystem);
    }

    public Command scoreInLowCone() {
        return collectionSubsystem.ejectCone();
    }

    public Command scoreInLowCube() {
        return collectionSubsystem.shootCube();
    }
}
