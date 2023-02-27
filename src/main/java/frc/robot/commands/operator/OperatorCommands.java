package frc.robot.commands.operator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.JointMovementType;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.JointsSetPosition;
import frc.robot.commands.operator.commands.Acquire;
import frc.robot.commands.operator.commands.Score;
import frc.robot.drivers.BeamBreak;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CollectionSubsystem;
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

    public Command goToHome(){
        return new JointsSetPosition(0, 0, 1, 0, armSubsystem, wristSubsystem);
    }
    
    public Command acquireConeFromFloor() {
        return new Acquire(ArmConstants.ACQUIRE_FROM_FLOOR, WristConstants.ACQUIRE_FROM_FLOOR, true, armSubsystem, wristSubsystem, collectionSubsystem);
    }

    public Command acquireCubeFromFloor() {
        return new Acquire(ArmConstants.ACQUIRE_FROM_FLOOR, WristConstants.ACQUIRE_FROM_FLOOR, false, armSubsystem, wristSubsystem, collectionSubsystem);
    }

    public Command acquireConeFromDoS() {
        return new Acquire(ArmConstants.ACQUIRE_FROM_DOS, WristConstants.ACQUIRE_FROM_DOS, true, armSubsystem, wristSubsystem, collectionSubsystem);
    }

    public Command acquireCubeFromDoS() {
        return new Acquire(ArmConstants.ACQUIRE_FROM_DOS, WristConstants.ACQUIRE_FROM_DOS, false, armSubsystem, wristSubsystem, collectionSubsystem);
    }

    public Command scoreInHighCone() {
        return new Score(ArmConstants.SCORE_IN_HIGH_CONE, WristConstants.SCORE_IN_HIGH_CONE, true, armSubsystem, wristSubsystem, collectionSubsystem);
    }

    public Command scoreInHighCube() {
        return new Score(ArmConstants.SCORE_IN_HIGH_CUBE, WristConstants.SCORE_IN_HIGH_CUBE, false, armSubsystem, wristSubsystem, collectionSubsystem);
    }

    public Command scoreInMidCone() {
        return new Score(ArmConstants.SCORE_IN_MID_CONE, WristConstants.SCORE_IN_MID_CONE, true, armSubsystem, wristSubsystem, collectionSubsystem);
    }

    public Command scoreInMidCube() {
        return new Score(ArmConstants.SCORE_IN_MID_CUBE, WristConstants.SCORE_IN_MID_CUBE, false, armSubsystem, wristSubsystem, collectionSubsystem);
    }

    public Command scoreInLowCone() {
        return collectionSubsystem.ejectCone();
    }

    public Command scoreInLowCube() {
        return collectionSubsystem.shootCube();
    }
}
