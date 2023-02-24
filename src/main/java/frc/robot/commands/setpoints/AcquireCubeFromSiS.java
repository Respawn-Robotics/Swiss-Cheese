package frc.robot.commands.setpoints;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.JointMovementType;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.JointsSetPosition;
import frc.robot.drivers.BeamBreak;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CollectionSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class AcquireCubeFromSiS extends CommandBase {
    private WristSubsystem wristSubsystem;
    private ArmSubsystem armSubsystem;
    private CollectionSubsystem collectionSubsystem;
    private BeamBreak cubeBreak;

    public AcquireCubeFromSiS(ArmSubsystem armSubsystem, WristSubsystem wristSubsystem, CollectionSubsystem collectionSubsystem, BeamBreak cubeBeak) {
        this.armSubsystem = armSubsystem;
        this.wristSubsystem = wristSubsystem;
        this.collectionSubsystem = collectionSubsystem;
        this.cubeBreak = cubeBeak;
    }

    @Override
    public void execute() {
        new JointsSetPosition(ArmConstants.ACQUIRE_FROM_SIS, WristConstants.ACUQIRE_FROM_SIS, JointMovementType.WRIST_FIRST, 0.4, armSubsystem, wristSubsystem)
        .andThen();
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.setVoltage(0).schedule();
    }
}
