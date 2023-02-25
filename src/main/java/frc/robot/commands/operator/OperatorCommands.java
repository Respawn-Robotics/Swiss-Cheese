package frc.robot.commands.operator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.JointMovementType;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.JointsSetPosition;

public class OperatorCommands {
    
    public static Command acquireConeFromFloor() {
        return new JointsSetPosition(ArmConstants.ACQUIRE_FROM_FLOOR, 
        WristConstants.ACQUIRE_FROM_FLOOR, 
        JointMovementType.WRIST_FIRST, 
        0.4)
        .andThen(RobotContainer.collectionSubsystem.collectCone());
    }

    public static Command acquireCubeFromFloor() {
        return new JointsSetPosition(ArmConstants.ACQUIRE_FROM_FLOOR, 
        WristConstants.ACQUIRE_FROM_FLOOR, 
        JointMovementType.WRIST_FIRST, 
        0.4)
        .andThen(RobotContainer.collectionSubsystem.collectCube());
    }

    public static Command acquireConeFromDoS() {
        return new JointsSetPosition(ArmConstants.ACQUIRE_FROM_DOS, 
        WristConstants.ACQUIRE_FROM_DOS, 
        JointMovementType.WRIST_FIRST,
        0.4)
        .andThen(RobotContainer.collectionSubsystem.collectCone());
    }

    public static Command acquireCubeFromDoS() {
        return new JointsSetPosition(ArmConstants.ACQUIRE_FROM_DOS, 
        WristConstants.ACQUIRE_FROM_DOS, 
        JointMovementType.WRIST_FIRST,
        0.4)
        .andThen(RobotContainer.collectionSubsystem.collectCube());
    }

    public static Command scoreInHighCone() {
        return new JointsSetPosition(ArmConstants.SCORE_IN_HIGH_CONE, 
        WristConstants.ACQUIRE_FROM_DOS, 
        JointMovementType.WRIST_FIRST,
        0.4)
        .andThen(RobotContainer.collectionSubsystem.ejectCone());
    }

    public static Command scoreInHighCube() {
        return new JointsSetPosition(ArmConstants.SCORE_IN_HIGH_CONE, 
        WristConstants.ACQUIRE_FROM_DOS, 
        JointMovementType.WRIST_FIRST,
        0.4)
        .andThen(RobotContainer.collectionSubsystem.shootCube());
    }

    public static Command scoreInMidCone() {
        return new JointsSetPosition(ArmConstants.SCORE_IN_MID_CONE, 
        WristConstants.SCORE_IN_MID_CONE, 
        JointMovementType.WRIST_FIRST,
        0.4)
        .andThen(RobotContainer.collectionSubsystem.ejectCone());
    }

    public static Command scoreInMidCube() {
        return new JointsSetPosition(ArmConstants.SCORE_IN_MID_CUBE, 
        WristConstants.SCORE_IN_MID_CUBE, 
        JointMovementType.WRIST_FIRST,
        0.4)
        .andThen(RobotContainer.collectionSubsystem.shootCube());
    }

    public static Command scoreInLowCone() {
        return RobotContainer.collectionSubsystem.ejectCone();
    }

    public static Command scoreInLowCube() {
        return RobotContainer.collectionSubsystem.shootCube();
    }
}
