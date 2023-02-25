package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.JointMovementType;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.JointsSetPosition;
import frc.robot.drivers.BeamBreak;

public class Superstructure extends SubsystemBase {
    
    private BeamBreak cubeBeamBreak = new BeamBreak(1);
    private BeamBreak coneBeamBreak = new BeamBreak(2);

    @Override
    public void periodic() {
        cubeBeamBreak.update();
        coneBeamBreak.update();

        if(cubeBeamBreak.wasTripped() || coneBeamBreak.wasTripped()) {
            new WaitCommand(0.5)
                .andThen(new JointsSetPosition(ArmConstants.HOME, WristConstants.HOME, JointMovementType.WRIST_FIRST, 0.4));
        }
    }
}
